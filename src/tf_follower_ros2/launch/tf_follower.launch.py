from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="tf_follower_ros2",
                executable="tf_follower",
                name="tf_follower",
                output="screen",
                parameters=[
                    {
                        "follower_frame": "mybot_link",
                        "target_frame": "base_footprint",
                        "cmd_vel_topic": "/mybot_cmd_vel",
                        "stop_distance": 1.0,
                        "linear_gain": 0.1,
                        "angular_gain": -0.4,
                        "max_linear_speed": 1.0,
                        "max_angular_speed": 1.5,
                        "lookup_rate_hz": 10.0,
                    }
                ],
            )
        ]
    )
