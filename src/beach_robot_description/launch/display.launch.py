from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('beach_robot_description')
    xacro_path = PathJoinSubstitution([pkg_share, 'urdf', 'beach_robot.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'display.rviz'])

    robot_description = Command(['xacro ', xacro_path])

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='Launch joint_state_publisher_gui'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=None,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
