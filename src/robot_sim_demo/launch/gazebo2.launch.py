"""Launch the safety inspection robot in the ISCAS Museum Gazebo world."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ROBOT_NAME = "wheeltec_robot"
WORLD_NAME = "default"


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("robot_sim_demo"))
    default_world = share / "worlds" / "museum.sdf"
    robot_sdf = share / "models" / ROBOT_NAME / "model.sdf"
    robot_urdf = share / "wheeltec_robot_urdf" / "urdf" / "mini_akm_robot.urdf"
    bridge_config = share / "config" / "gazebo2_bridge.yaml"
    default_gui_config = share / "gui" / "museum.gui.config"
    default_rviz_config = share / "rviz" / "museum.rviz"

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    spawn_robot = LaunchConfiguration("spawn_robot")
    drive = LaunchConfiguration("drive")
    drive_linear_speed = LaunchConfiguration("drive_linear_speed")
    drive_angular_speed = LaunchConfiguration("drive_angular_speed")
    drive_loop = LaunchConfiguration("drive_loop")
    drive_duration = LaunchConfiguration("drive_duration")
    gz_partition = LaunchConfiguration("gz_partition")
    world_name = LaunchConfiguration("world_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_path = LaunchConfiguration("world")
    gui_config = LaunchConfiguration("gui_config")
    rviz_config = LaunchConfiguration("rviz_config")

    robot_description = robot_urdf.read_text(encoding="utf-8")

    gz_executable = "gz"
    gazebo_gui = ExecuteProcess(
        cmd=[
            gz_executable, "sim", "-r",
            "--gui-config", gui_config,
            world_path,
        ],
        name="gazebo",
        output="screen",
        condition=IfCondition(gui),
        additional_env={
            "GZ_PARTITION": gz_partition,
            "GZ_SIM_RESOURCE_PATH": os.pathsep.join(
                filter(None, [
                    str(share / "models"),
                    str(share / "wheeltec_robot_urdf"),
                    os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
                ])
            ),
            "IGN_GAZEBO_RESOURCE_PATH": os.pathsep.join(
                filter(None, [
                    str(share / "models"),
                    str(share / "wheeltec_robot_urdf"),
                    os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
                ])
            ),
        },
    )
    gazebo_headless = ExecuteProcess(
        cmd=[
            gz_executable, "sim", "-r", "-s", "--headless-rendering",
            world_path,
        ],
        name="gazebo",
        output="screen",
        condition=UnlessCondition(gui),
        additional_env={
            "GZ_PARTITION": gz_partition,
            "GZ_SIM_RESOURCE_PATH": os.pathsep.join(
                filter(None, [
                    str(share / "models"),
                    str(share / "wheeltec_robot_urdf"),
                    os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
                ])
            ),
            "IGN_GAZEBO_RESOURCE_PATH": os.pathsep.join(
                filter(None, [
                    str(share / "models"),
                    str(share / "wheeltec_robot_urdf"),
                    os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
                ])
            ),
        },
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        condition=IfCondition(spawn_robot),
        output="screen",
        arguments=[
            "-world",
            world_name,
            "-file",
            str(robot_sdf),
            "-name",
            ROBOT_NAME,
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-Y",
            spawn_yaw,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("spawn_robot", default_value="true"),
            DeclareLaunchArgument("drive", default_value="true"),
            DeclareLaunchArgument("drive_linear_speed", default_value="0.18"),
            DeclareLaunchArgument("drive_angular_speed", default_value="0.55"),
            DeclareLaunchArgument("drive_loop", default_value="true"),
            DeclareLaunchArgument("drive_duration", default_value="0.0"),
            DeclareLaunchArgument("gz_partition", default_value="robot_sim_demo"),
            DeclareLaunchArgument("world", default_value=str(default_world)),
            DeclareLaunchArgument("world_name", default_value=WORLD_NAME),
            DeclareLaunchArgument("gui_config", default_value=str(default_gui_config)),
            DeclareLaunchArgument("rviz_config", default_value=str(default_rviz_config)),
            SetEnvironmentVariable(name="GZ_PARTITION", value=gz_partition),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.017"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    str(share / "models"),
                    os.pathsep,
                    str(share / "wheeltec_robot_urdf"),
                    os.pathsep,
                    os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
                ],
            ),
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=[
                    str(share / "models"),
                    os.pathsep,
                    str(share / "wheeltec_robot_urdf"),
                    os.pathsep,
                    os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
                ],
            ),
            gazebo_gui,
            gazebo_headless,
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gazebo2_bridge",
                output="screen",
                parameters=[
                    {
                        "config_file": str(bridge_config),
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="ros_gz_image",
                executable="image_bridge",
                name="gazebo2_camera_bridge",
                output="screen",
                arguments=["/camera/image_raw"],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="robot_sim_demo",
                executable="camera_info_publisher",
                name="camera_info_publisher",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="gazebo2_robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="robot_sim_demo",
                executable="patrol_driver",
                name="patrol_driver",
                condition=IfCondition(drive),
                output="screen",
                parameters=[
                    {"linear_speed": drive_linear_speed},
                    {"angular_speed": drive_angular_speed},
                    {"loop": drive_loop},
                    {"duration": drive_duration},
                ],
            ),
            TimerAction(period=3.0, actions=[spawn]),
            Node(
                package="rviz2",
                executable="rviz2",
                name="museum_rviz",
                condition=IfCondition(rviz),
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
