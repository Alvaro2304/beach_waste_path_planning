"""Launch the beach-robot custom EKF (IMU + GPS fusion)."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('beach_robot_custom_ekf')
    default_params = os.path.join(pkg_dir, 'config', 'ekf_params.yaml')

    return LaunchDescription([
        # Allow the user to override the parameter file at launch time.
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the EKF parameter YAML file',
        ),

        Node(
            package='beach_robot_custom_ekf',
            executable='ekf_imu_gps',
            name='ekf_imu_gps_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
