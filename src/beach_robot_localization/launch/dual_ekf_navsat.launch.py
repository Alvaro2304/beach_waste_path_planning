"""
Launch file for beach_robot_localization.

Starts two robot_localization nodes:
  1. ekf_node          -- fuses IMU + GPS odometry into /odometry/filtered
  2. navsat_transform   -- converts /gps/fix to /odometry/gps in the odom frame

Usage:
  ros2 launch beach_robot_localization dual_ekf_navsat.launch.py
  ros2 launch beach_robot_localization dual_ekf_navsat.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # -----------------------------------------------------------------------
    # Paths
    # -----------------------------------------------------------------------
    pkg_share = get_package_share_directory('beach_robot_localization')

    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    navsat_config = os.path.join(pkg_share, 'config', 'navsat.yaml')

    # -----------------------------------------------------------------------
    # Launch arguments
    # -----------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # -----------------------------------------------------------------------
    # Nodes
    # -----------------------------------------------------------------------

    # EKF node -- Extended Kalman Filter
    # Subscribes to: /imu/data, /odometry/gps
    # Publishes:     /odometry/filtered, TF odom -> base_link
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ],
    )

    # NavSat Transform node -- GPS to local odometry conversion
    # Subscribes to: /gps/fix, /imu/data, /odometry/filtered
    # Publishes:     /odometry/gps, /gps/filtered, TF utm -> odom (optional)
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
            navsat_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('gps/fix', '/gps/fix'),
            ('imu', '/imu/data'),
            ('odometry/filtered', '/odometry/filtered'),
            ('odometry/gps', '/odometry/gps'),
            ('gps/filtered', '/gps/filtered'),
        ],
    )

    # -----------------------------------------------------------------------
    # Launch description
    # -----------------------------------------------------------------------
    return LaunchDescription([
        use_sim_time_arg,
        ekf_node,
        navsat_transform_node,
    ])
