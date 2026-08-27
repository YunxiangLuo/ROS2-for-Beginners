"""Start MoveIt 2 for the standalone xArm simulation."""
from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def _load_yaml(filename: str) -> dict:
    with open(filename, encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("xarm_ros2_arm_only")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_description = LaunchConfiguration("publish_robot_description")
    base_height = LaunchConfiguration("base_height")

    # Gazebo, MoveIt, and RViz share the same fixed world_to_base transform.
    moveit_config = (
        MoveItConfigsBuilder("xarm", package_name="xarm_ros2_arm_only")
        .robot_description(
            file_path=os.path.join(package_share, "urdf", "arm_only_xarm.urdf.xacro"),
            mappings={
                "hardware_type": "mock_components",
                "use_ros2_control": "false",
                "include_world_joint": "true",
                "base_height": base_height,
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(package_share, "config", "xarm.srdf")
        )
        .robot_description_kinematics(
            file_path=os.path.join(package_share, "config", "arm_only_kinematics.yaml")
        )
        .joint_limits(
            file_path=os.path.join(package_share, "config", "arm_only_joint_limits.yaml")
        )
        .trajectory_execution(
            file_path=os.path.join(package_share, "config", "moveit_controllers.yaml"),
            moveit_manage_controllers=False,
        )
        .planning_pipelines(
            default_planning_pipeline="ompl", pipelines=["ompl"], load_all=False
        )
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=False,
        )
        .to_dict()
    )
    moveit_config["ompl"] = _load_yaml(
        os.path.join(package_share, "config", "arm_only_ompl_planning.yaml")
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("base_height", default_value="0.0"),
            DeclareLaunchArgument("publish_robot_description", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(package_share, "config", "arm_only_moveit.rviz"),
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                parameters=[
                    moveit_config,
                    {
                        "use_sim_time": use_sim_time,
                        "publish_robot_description": ParameterValue(
                            publish_robot_description, value_type=bool
                        ),
                    },
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="arm_only_rviz",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[moveit_config, {"use_sim_time": use_sim_time}],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                output="screen",
            ),
        ]
    )
