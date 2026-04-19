from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('beach_robot_description')
    default_config = PathJoinSubstitution([pkg_share, 'config', 'joy_ps4.yaml'])

    config_arg = DeclareLaunchArgument(
        'config', default_value=default_config,
        description='Path to joy + teleop_twist_joy YAML config'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Set true when driving the Gazebo sim'
    )

    config = LaunchConfiguration('config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([config_arg, use_sim_time_arg, joy_node, teleop_node])
