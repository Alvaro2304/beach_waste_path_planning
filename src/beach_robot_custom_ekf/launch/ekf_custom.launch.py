"""Launch the beach-robot custom EKF (IMU + GPS fusion).

Usage:
  ros2 launch beach_robot_custom_ekf ekf_custom.launch.py
  ros2 launch beach_robot_custom_ekf ekf_custom.launch.py use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('beach_robot_custom_ekf')
    default_params = os.path.join(pkg_dir, 'config', 'ekf_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the EKF parameter YAML file',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    ekf_node = Node(
        package='beach_robot_custom_ekf',
        executable='ekf_imu_gps',
        name='ekf_imu_gps_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([params_file_arg, use_sim_time_arg, ekf_node])
