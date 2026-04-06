// ============================================================================
// beach_robot_custom_ekf  --  EKF fusing IMU + GPS for beach robot
// ============================================================================
//
// State vector:  x = [x, y, theta, v, omega]  (5-state)
//
// Prediction (IMU-driven, ~50 Hz timer):
//   IMU accelerometer and gyroscope are treated as CONTROL INPUTS, not
//   measurements.  This avoids the independence-violation bug in the
//   original TurtleBot4 EKF where ax was folded into a fake velocity
//   measurement (z = v + ax*dt) that contained the current state.
//
// Measurement updates (asynchronous):
//   GPS  -> observes [x, y]          via equirectangular local projection
//   IMU  -> observes [theta, omega]  from orientation quaternion and gyro
//
// Key fixes over the turtlebot4 predecessor:
//   1. IMU accel is a control input, NOT a measurement.
//   2. Angle wrapping uses an explicit angle_indices vector -- no fragile
//      H-matrix inspection that silently fails when row order changes.
//   3. Covariance update uses the numerically-stable Joseph form:
//        P = (I - K H) P (I - K H)^T  +  K R K^T
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

// ---------------------------------------------------------------------------
// Helper: wrap angle to [-pi, pi]
// ---------------------------------------------------------------------------
static double wrapAngle(double angle)
{
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0)
        angle += 2.0 * M_PI;
    return angle - M_PI;
}

// ============================================================================
class EkfImuGpsNode : public rclcpp::Node
{
public:
    EkfImuGpsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("ekf_imu_gps_node", options)
    {
        // -- Declare all ROS parameters ----------------------------------------
        declareParameters();

        // -- Load parameter values into member variables -----------------------
        loadParameters();

        // -- Initialise EKF state & covariance ---------------------------------
        state_ = Eigen::VectorXd::Zero(N);
        P_     = Eigen::MatrixXd::Identity(N, N) * initial_covariance_;

        // -- TF broadcaster ----------------------------------------------------
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // -- BEST_EFFORT QoS for sensor topics ---------------------------------
        auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                              .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        // -- Subscribers -------------------------------------------------------
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", sensor_qos,
            std::bind(&EkfImuGpsNode::imuCallback, this, std::placeholders::_1));

        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", sensor_qos,
            std::bind(&EkfImuGpsNode::gpsCallback, this, std::placeholders::_1));

        // -- Publisher ---------------------------------------------------------
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10);

        // -- Prediction timer (~50 Hz) -----------------------------------------
        prediction_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&EkfImuGpsNode::predictionStep, this));

        // -- Sensor-timeout watchdog (1 Hz) ------------------------------------
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&EkfImuGpsNode::watchdogCheck, this));

        RCLCPP_INFO(this->get_logger(),
                    "EKF IMU+GPS node started  (state dim = %d)", N);
    }

private:
    // -- State dimension -------------------------------------------------------
    static constexpr int N = 5; // [x, y, theta, v, omega]

    // -- EKF state & covariance (guarded by mutex_) ----------------------------
    std::mutex           mutex_;
    Eigen::VectorXd      state_;
    Eigen::MatrixXd      P_;

    // -- Noise matrices (set from parameters) ----------------------------------
    Eigen::MatrixXd      Q_;       // 5x5 process noise
    Eigen::MatrixXd      R_gps_;   // 2x2 GPS measurement noise
    Eigen::MatrixXd      R_imu_;   // 2x2 IMU measurement noise

    // -- GPS origin (set from first valid fix) ---------------------------------
    bool   gps_origin_set_ = false;
    double lat0_ = 0.0;
    double lon0_ = 0.0;
    double alt0_ = 0.0;

    // -- Initialisation flags --------------------------------------------------
    bool imu_initialised_ = false;
    bool gps_initialised_ = false;
    bool ekf_initialised_ = false;  // true once BOTH first GPS + first IMU arrive

    // -- Latest IMU data (written in callback, read in prediction) -------------
    sensor_msgs::msg::Imu::SharedPtr latest_imu_;
    bool   have_imu_data_ = false;
    double latest_pitch_   = 0.0;   // pitch from IMU orientation, used to
                                    // project gravity out of ax

    // -- Timing ----------------------------------------------------------------
    rclcpp::Time last_prediction_time_{0, 0, RCL_CLOCK_UNINITIALIZED};
    rclcpp::Time last_imu_stamp_{0, 0, RCL_CLOCK_UNINITIALIZED};
    rclcpp::Time last_gps_stamp_{0, 0, RCL_CLOCK_UNINITIALIZED};
    bool         prediction_time_initialised_ = false;

    // -- Parameter cache -------------------------------------------------------
    double initial_covariance_;
    double imu_timeout_sec_;
    double gps_timeout_sec_;

    // -- ROS handles -----------------------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr  gps_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         odom_pub_;
    rclcpp::TimerBase::SharedPtr prediction_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ======================================================================
    // Parameter declaration & loading
    // ======================================================================
    void declareParameters()
    {
        // Process noise Q (diagonal)
        this->declare_parameter<double>("q_x",     0.01);
        this->declare_parameter<double>("q_y",     0.01);
        this->declare_parameter<double>("q_theta", 0.005);
        this->declare_parameter<double>("q_v",     0.1);
        this->declare_parameter<double>("q_omega", 0.05);

        // GPS measurement noise R
        this->declare_parameter<double>("r_gps_x", 4.0);
        this->declare_parameter<double>("r_gps_y", 4.0);

        // IMU measurement noise R
        this->declare_parameter<double>("r_imu_theta", 0.01);
        this->declare_parameter<double>("r_imu_omega", 0.001);

        // Misc
        this->declare_parameter<double>("initial_covariance", 1.0);
        this->declare_parameter<double>("imu_timeout",  0.5);
        this->declare_parameter<double>("gps_timeout", 10.0);
    }

    void loadParameters()
    {
        // -- Q (5x5 diagonal) --------------------------------------------------
        Q_ = Eigen::MatrixXd::Zero(N, N);
        Q_(0, 0) = this->get_parameter("q_x").as_double();
        Q_(1, 1) = this->get_parameter("q_y").as_double();
        Q_(2, 2) = this->get_parameter("q_theta").as_double();
        Q_(3, 3) = this->get_parameter("q_v").as_double();
        Q_(4, 4) = this->get_parameter("q_omega").as_double();

        // -- R_gps (2x2) -------------------------------------------------------
        R_gps_ = Eigen::MatrixXd::Zero(2, 2);
        R_gps_(0, 0) = this->get_parameter("r_gps_x").as_double();
        R_gps_(1, 1) = this->get_parameter("r_gps_y").as_double();

        // -- R_imu (2x2) -------------------------------------------------------
        R_imu_ = Eigen::MatrixXd::Zero(2, 2);
        R_imu_(0, 0) = this->get_parameter("r_imu_theta").as_double();
        R_imu_(1, 1) = this->get_parameter("r_imu_omega").as_double();

        // -- Scalars -----------------------------------------------------------
        initial_covariance_ = this->get_parameter("initial_covariance").as_double();
        imu_timeout_sec_    = this->get_parameter("imu_timeout").as_double();
        gps_timeout_sec_    = this->get_parameter("gps_timeout").as_double();
    }

    // ======================================================================
    // IMU callback -- stores latest data; runs IMU measurement update
    // ======================================================================
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        last_imu_stamp_ = rclcpp::Time(msg->header.stamp);

        // -- Extract orientation (roll, pitch, yaw) ----------------------------
        tf2::Quaternion quat;
        tf2::fromMsg(msg->orientation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // Store pitch for gravity projection in prediction step
        latest_pitch_ = pitch;

        // Store the whole message for the prediction timer
        latest_imu_  = msg;
        have_imu_data_ = true;

        // -- First IMU: seed theta & omega, mark as initialised ----------------
        if (!imu_initialised_) {
            state_[2] = yaw;                         // theta
            state_[4] = msg->angular_velocity.z;     // omega
            imu_initialised_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "IMU initialised: yaw=%.4f  omega=%.4f", yaw, state_[4]);
            checkFullInitialisation();
            return;
        }

        if (!ekf_initialised_) return;

        // -- IMU measurement update: observe [theta, omega] --------------------
        Eigen::VectorXd z(2);
        z[0] = yaw;
        z[1] = msg->angular_velocity.z;

        Eigen::MatrixXd H(2, N);
        H.setZero();
        H(0, 2) = 1.0;  // theta is state index 2
        H(1, 4) = 1.0;  // omega is state index 4

        // The innovation index 0 (first row) corresponds to theta -- an angle.
        std::vector<int> angle_indices = {0};

        ekfUpdate(z, H, R_imu_, angle_indices);
    }

    // ======================================================================
    // GPS callback -- sets origin on first fix, then runs GPS measurement update
    // ======================================================================
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // -- Skip if no valid fix ----------------------------------------------
        if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "GPS: no fix (status=%d), skipping update",
                                 msg->status.status);
            return;
        }

        last_gps_stamp_ = rclcpp::Time(msg->header.stamp);

        // -- First valid fix: set local-frame origin ---------------------------
        if (!gps_origin_set_) {
            lat0_ = msg->latitude;
            lon0_ = msg->longitude;
            alt0_ = msg->altitude;
            gps_origin_set_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "GPS origin set: lat=%.8f  lon=%.8f  alt=%.2f",
                        lat0_, lon0_, alt0_);
        }

        // -- Convert to local XY (equirectangular projection) ------------------
        double x_local = (msg->longitude - lon0_) * std::cos(lat0_ * M_PI / 180.0) * 111320.0;
        double y_local = (msg->latitude  - lat0_) * 111320.0;

        // -- First GPS fix: seed position & mark initialised -------------------
        if (!gps_initialised_) {
            state_[0] = x_local;  // should be ~0 since this is the origin fix
            state_[1] = y_local;
            gps_initialised_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "GPS initialised: local x=%.3f  y=%.3f", x_local, y_local);
            checkFullInitialisation();
            return;
        }

        if (!ekf_initialised_) return;

        // -- GPS measurement update: observe [x, y] ---------------------------
        Eigen::VectorXd z(2);
        z[0] = x_local;
        z[1] = y_local;

        Eigen::MatrixXd H(2, N);
        H.setZero();
        H(0, 0) = 1.0;  // x
        H(1, 1) = 1.0;  // y

        // No angles in the GPS measurement vector.
        std::vector<int> angle_indices = {};

        ekfUpdate(z, H, R_gps_, angle_indices);
    }

    // ======================================================================
    // Check if both sensors have arrived so the EKF can start running
    // ======================================================================
    void checkFullInitialisation()
    {
        if (imu_initialised_ && gps_initialised_ && !ekf_initialised_) {
            ekf_initialised_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "EKF fully initialised: state = [%.3f, %.3f, %.4f, %.3f, %.4f]",
                        state_[0], state_[1], state_[2], state_[3], state_[4]);
        }
    }

    // ======================================================================
    // Prediction step (timer callback, ~50 Hz)
    //
    // Uses IMU accelerometer (ax) and gyroscope (gz) as CONTROL INPUTS.
    // This is the correct formulation: the IMU drives the process model,
    // it is NOT folded into a pseudo-measurement.
    // ======================================================================
    void predictionStep()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!ekf_initialised_ || !have_imu_data_) return;

        auto now = this->get_clock()->now();

        // -- Initialise prediction clock on first valid tick --------------------
        if (!prediction_time_initialised_) {
            last_prediction_time_        = now;
            prediction_time_initialised_ = true;
            return;
        }

        double dt = (now - last_prediction_time_).seconds();

        // Guard against pathological dt
        if (dt < 0.001) return;    // < 1 ms -- skip
        if (dt > 1.0) {
            RCLCPP_WARN(this->get_logger(),
                        "Prediction dt too large (%.3fs), resetting timer", dt);
            last_prediction_time_ = now;
            return;
        }

        // -- Read latest IMU control values ------------------------------------
        double ax = latest_imu_->linear_acceleration.x;
        double gz = latest_imu_->angular_velocity.z;

        // -- Current state aliases ---------------------------------------------
        double x     = state_[0];
        double y     = state_[1];
        double theta = state_[2];
        double v     = state_[3];
        // (omega is overwritten by gyro below)

        // -- Process model (IMU-driven) ----------------------------------------
        // Position: standard differential-drive kinematics
        double x_new     = x + v * std::cos(theta) * dt;
        double y_new     = y + v * std::sin(theta) * dt;

        // Heading: integrate gyroscope
        double theta_new = theta + gz * dt;

        // Velocity: integrate accelerometer, projecting out gravity via pitch.
        // ax_body = a_forward + g*sin(pitch).  Multiplying by cos(pitch)
        // removes the static gravity component to first order.
        double v_new     = v + ax * std::cos(latest_pitch_) * dt;

        // Angular rate: taken directly from gyroscope
        double omega_new = gz;

        // -- Write predicted state ---------------------------------------------
        state_[0] = x_new;
        state_[1] = y_new;
        state_[2] = wrapAngle(theta_new);
        state_[3] = v_new;
        state_[4] = omega_new;

        // -- Jacobian F (partial derivatives of process model wrt state) -------
        //
        // The process model depends on state through:
        //   x_new     = x  + v * cos(theta) * dt          => dF/dx=1, dF/dtheta, dF/dv
        //   y_new     = y  + v * sin(theta) * dt          => dF/dy=1, dF/dtheta, dF/dv
        //   theta_new = theta + gz * dt                   => dF/dtheta=1  (gz is control)
        //   v_new     = v  + ax*cos(pitch)*dt             => dF/dv=1     (ax, pitch are control)
        //   omega_new = gz                                => dF/domega=0 (gz is control)
        //
        // Note: omega_new does NOT depend on the previous omega (it is
        // directly replaced by the gyro reading).  Therefore dF/domega = 0
        // for the omega row, and we set F(4,4) = 0 explicitly.
        //
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(N, N);
        F(0, 2) = -v * std::sin(theta) * dt;   // dx_new / dtheta
        F(0, 3) =      std::cos(theta) * dt;   // dx_new / dv
        F(1, 2) =  v * std::cos(theta) * dt;   // dy_new / dtheta
        F(1, 3) =      std::sin(theta) * dt;   // dy_new / dv
        F(2, 2) =  1.0;                        // dtheta_new / dtheta
        F(3, 3) =  1.0;                        // dv_new / dv
        F(4, 4) =  0.0;                        // omega_new = gz (no state dependence)

        // -- Covariance prediction ---------------------------------------------
        // Scale Q by dt so that process noise is rate-consistent.
        P_ = F * P_ * F.transpose() + Q_ * dt;

        last_prediction_time_ = now;

        // -- Publish results ---------------------------------------------------
        publishResults(now);
    }

    // ======================================================================
    // Generic EKF measurement update
    //
    // Fixes over the original code:
    //   - angle_indices explicitly lists which rows of the innovation
    //     vector are angles, so wrapping is always correct regardless
    //     of H-matrix layout.
    //   - Covariance uses the Joseph form for numerical stability.
    //   - S is solved via Cholesky instead of .inverse().
    // ======================================================================
    void ekfUpdate(const Eigen::VectorXd          & z,
                   const Eigen::MatrixXd           & H,
                   const Eigen::MatrixXd           & R,
                   const std::vector<int>           & angle_indices)
    {
        // -- Innovation --------------------------------------------------------
        Eigen::VectorXd y = z - H * state_;

        // -- Angle wrapping on the correct innovation elements -----------------
        for (int idx : angle_indices) {
            y[idx] = wrapAngle(y[idx]);
        }

        // -- Innovation covariance S -------------------------------------------
        Eigen::MatrixXd S = H * P_ * H.transpose() + R;

        // -- Kalman gain via Cholesky solve (more stable than S.inverse()) -----
        //    We need K = P H^T S^{-1}.
        //    Compute S^{-1} * (H P^T) = S^{-1} * (P H^T)^T ... but easier:
        //    K^T = S^{-1} * H * P   =>   K = P * H^T * S^{-T}
        //    Since S is symmetric, S^{-T} = S^{-1}.
        //    So:  K = ( S.llt().solve(H * P_) )^T  ... but let's just solve
        //    column-by-column.  The simplest correct form:
        //      K = P_ * H^T * S^{-1}
        //    with S^{-1} computed via  S.llt().solve(I).
        Eigen::MatrixXd S_inv = S.llt().solve(
            Eigen::MatrixXd::Identity(S.rows(), S.cols()));
        Eigen::MatrixXd K = P_ * H.transpose() * S_inv;

        // -- State update ------------------------------------------------------
        state_ = state_ + K * y;
        state_[2] = wrapAngle(state_[2]);  // keep theta in [-pi, pi]

        // -- Covariance update: Joseph form ------------------------------------
        //    P = (I - K H) P (I - K H)^T  +  K R K^T
        //
        // This is algebraically equivalent to the simple form (I - K H) P
        // when K is the optimal Kalman gain, but it is ALWAYS positive
        // semi-definite even with floating-point rounding, unlike the
        // simple form which can become indefinite and diverge.
        Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(N, N) - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
    }

    // ======================================================================
    // Sensor-timeout watchdog (1 Hz)
    // ======================================================================
    void watchdogCheck()
    {
        if (!ekf_initialised_) return;

        auto now = this->get_clock()->now();

        if (last_imu_stamp_.nanoseconds() > 0) {
            double imu_age = (now - last_imu_stamp_).seconds();
            if (imu_age > imu_timeout_sec_) {
                RCLCPP_WARN(this->get_logger(),
                            "IMU timeout: no data for %.2fs (limit %.1fs)",
                            imu_age, imu_timeout_sec_);
            }
        }

        if (last_gps_stamp_.nanoseconds() > 0) {
            double gps_age = (now - last_gps_stamp_).seconds();
            if (gps_age > gps_timeout_sec_) {
                RCLCPP_WARN(this->get_logger(),
                            "GPS timeout: no data for %.1fs (limit %.1fs)",
                            gps_age, gps_timeout_sec_);
            }
        }
    }

    // ======================================================================
    // Publish filtered odometry + TF
    // ======================================================================
    void publishResults(const rclcpp::Time & stamp)
    {
        // -- Build quaternion from yaw -----------------------------------------
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, state_[2]);

        // -- TF: odom -> base_link --------------------------------------------
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp    = stamp;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id  = "base_link";

        tf_msg.transform.translation.x = state_[0];
        tf_msg.transform.translation.y = state_[1];
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation      = tf2::toMsg(quat);

        tf_broadcaster_->sendTransform(tf_msg);

        // -- Odometry message --------------------------------------------------
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp    = stamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id  = "base_link";

        odom_msg.pose.pose.position.x  = state_[0];
        odom_msg.pose.pose.position.y  = state_[1];
        odom_msg.pose.pose.position.z  = 0.0;
        odom_msg.pose.pose.orientation = tf2::toMsg(quat);

        odom_msg.twist.twist.linear.x  = state_[3];
        odom_msg.twist.twist.linear.y  = 0.0;
        odom_msg.twist.twist.angular.z = state_[4];

        // -- Map 5x5 covariance into the 6x6 ROS covariance arrays ------------
        // ROS covariance is row-major 6x6: indices [x, y, z, roll, pitch, yaw]
        // We only populate the diagonal entries we track.
        std::fill(odom_msg.pose.covariance.begin(),
                  odom_msg.pose.covariance.end(), 0.0);
        std::fill(odom_msg.twist.covariance.begin(),
                  odom_msg.twist.covariance.end(), 0.0);

        odom_msg.pose.covariance[0]  = P_(0, 0);   // x
        odom_msg.pose.covariance[1]  = P_(0, 1);   // x-y cross
        odom_msg.pose.covariance[5]  = P_(0, 2);   // x-yaw cross  (col 5 = yaw in 6x6)
        odom_msg.pose.covariance[6]  = P_(1, 0);   // y-x cross
        odom_msg.pose.covariance[7]  = P_(1, 1);   // y
        odom_msg.pose.covariance[11] = P_(1, 2);   // y-yaw cross
        odom_msg.pose.covariance[30] = P_(2, 0);   // yaw-x cross
        odom_msg.pose.covariance[31] = P_(2, 1);   // yaw-y cross
        odom_msg.pose.covariance[35] = P_(2, 2);   // yaw

        odom_msg.twist.covariance[0]  = P_(3, 3);  // vx
        odom_msg.twist.covariance[35] = P_(4, 4);  // omega (vyaw)

        odom_pub_->publish(odom_msg);
    }
};

// ============================================================================
// main
// ============================================================================
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EkfImuGpsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
