from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    cmd_vel_topic = "/mybot_cmd_vel"
    follower_frame = "mybot_link"
    target_frame = "base_footprint"

    return LaunchDescription(
        [
            Node(
                package="tf_follower_ros2",
                executable="fake_target_broadcaster",
                name="fake_target_broadcaster",
                output="screen",
                parameters=[
                    {
                        "parent_frame": follower_frame,
                        "child_frame": target_frame,
                        "motion_mode": "circle",
                        "z": 0.0,
                        "center_x": 3.0,
                        "center_y": 0.0,
                        "radius": 1.0,
                        "angular_speed": 0.5,
                        "period_sec": 0.1,
                    }
                ],
            ),
            Node(
                package="tf_follower_ros2",
                executable="tf_follower",
                name="tf_follower",
                output="screen",
                parameters=[
                    {
                        "follower_frame": follower_frame,
                        "target_frame": target_frame,
                        "cmd_vel_topic": cmd_vel_topic,
                        "stop_distance": 1.0,
                        "linear_gain": 0.1,
                        "angular_gain": -0.4,
                        "max_linear_speed": 1.0,
                        "max_angular_speed": 1.5,
                        "lookup_rate_hz": 10.0,
                    }
                ],
            ),
        ]
    )
