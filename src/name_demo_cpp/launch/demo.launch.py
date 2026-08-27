from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="name_demo_cpp",
                executable="name_demo_node",
                namespace="academy",
                name="name_demo",
                parameters=[
                    {"serial": 10},
                    {"global_serial": 5},
                ],
                output="screen",
            )
        ]
    )
