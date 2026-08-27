import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_root = os.environ["PIXI_PROJECT_ROOT"]
    package_share = get_package_share_directory("urdf_demo_ros2")
    default_model = os.path.join(project_root, "urdf_demo", "urdf", "mycar_color_geometry.urdf")
    default_rviz = os.path.join(package_share, "rviz", "base_link.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value=default_model),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": Command(["xacro ", LaunchConfiguration("model")])}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(LaunchConfiguration("use_gui")),
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                output="screen",
            ),
        ]
    )
