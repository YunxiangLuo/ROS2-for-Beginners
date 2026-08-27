"""Start the lifecycle example with optional automatic activation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'autostart',
            default_value='false',
            description='Whether to configure and activate the lifecycle node',
        ),
        Node(
            package='lifecycle_demo',
            executable='hello_node',
            name='hello_ros2_lifecycle',
            output='screen',
            parameters=[{
                'autostart': LaunchConfiguration('autostart'),
            }],
        ),
    ])
