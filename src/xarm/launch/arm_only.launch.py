"""Launch the xArm6 alone in Gazebo Harmonic with MoveIt and RViz."""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _enabled(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "on"}


def _launch_setup(context):
    package_share = get_package_share_directory("xarm_ros2_arm_only")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    xarm_description_share = get_package_share_directory("xarm_description")

    controllers_file = os.path.join(package_share, "config", "arm_only_controllers.yaml")
    xacro_file = os.path.join(package_share, "urdf", "arm_only_xarm.urdf.xacro")
    gui_config = LaunchConfiguration("gazebo_gui_config").perform(context)
    world = LaunchConfiguration("world").perform(context)
    headless = _enabled(LaunchConfiguration("gz_headless").perform(context))

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    xacro_file,
                    " hardware_type:=gz",
                    " use_ros2_control:=true",
                    " controllers_file:=",
                    controllers_file,
                    " include_world_joint:=true",
                    " base_height:=",
                    LaunchConfiguration("base_height"),
                ]
            ),
            value_type=str,
        )
    }
    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    if headless:
        gz_args = f'-r -s --headless-rendering -v 3 "{world}"'
    else:
        gz_args = f'-r --gui-config "{gui_config}" -v 3 "{world}"'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
        condition=IfCondition(LaunchConfiguration("use_gazebo")),
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="arm_only_robot_state_publisher",
        parameters=[robot_description, use_sim_time],
        output="screen",
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="arm_only_clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        condition=IfCondition(LaunchConfiguration("use_gazebo")),
        output="screen",
    )

    spawn_xarm = TimerAction(
        period=2.0,
        condition=IfCondition(LaunchConfiguration("use_gazebo")),
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_arm_only_xarm",
                arguments=[
                    "-world",
                    LaunchConfiguration("world_name"),
                    "-topic",
                    "robot_description",
                    "-name",
                    "xarm",
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.0",
                    "-Y",
                    "0.0",
                ],
                parameters=[use_sim_time],
                output="screen",
            )
        ],
    )

    controller_spawner_args = [
        "--controller-manager",
        "/controller_manager",
        "--controller-manager-timeout",
        "60",
        "--service-call-timeout",
        "30",
    ]
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", *controller_spawner_args],
        parameters=[use_sim_time],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["xarm_controller", *controller_spawner_args],
        parameters=[use_sim_time],
        output="screen",
    )
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", *controller_spawner_args],
        parameters=[use_sim_time],
        output="screen",
    )
    controller_start = TimerAction(
        period=4.0,
        condition=IfCondition(LaunchConfiguration("use_gazebo")),
        actions=[joint_state_spawner],
    )
    # When Gazebo is disabled, MoveIt uses mock hardware with no
    # controller_manager, so publish /joint_states from a GUI-less
    # joint_state_publisher to keep robot_state_publisher fed.
    mock_joint_state = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="arm_only_joint_state_publisher",
        parameters=[use_sim_time],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_gazebo")),
    )

    move_group = TimerAction(
        period=3.0,
        condition=IfCondition(LaunchConfiguration("run_move_group")),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_share, "launch", "arm_only_move_group.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "base_height": LaunchConfiguration("base_height"),
                    "publish_robot_description": "false",
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "rviz_config": LaunchConfiguration("rviz_config"),
                }.items(),
            )
        ],
    )

    return [
        gazebo,
        state_publisher,
        clock_bridge,
        spawn_xarm,
        controller_start,
        mock_joint_state,
        RegisterEventHandler(
            OnProcessExit(target_action=joint_state_spawner, on_exit=[arm_spawner])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=arm_spawner, on_exit=[gripper_spawner])
        ),
        move_group,
    ]


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("xarm_ros2_arm_only")
    xarm_description_share = get_package_share_directory("xarm_description")
    gz_ros2_control_lib = os.path.join(get_package_prefix("gz_ros2_control"), "lib")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(package_share, "worlds", "arm_only.sdf"),
            ),
            DeclareLaunchArgument("world_name", default_value="arm_only"),
            DeclareLaunchArgument(
                "gazebo_gui_config",
                default_value=os.path.join(package_share, "config", "arm_only_gazebo.config"),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(package_share, "config", "arm_only_moveit.rviz"),
            ),
            DeclareLaunchArgument("base_height", default_value="0.0"),
            DeclareLaunchArgument("use_gazebo", default_value="true"),
            DeclareLaunchArgument("gz_headless", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("run_move_group", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rmw_implementation", default_value="rmw_cyclonedds_cpp"),
            SetEnvironmentVariable(
                "RMW_IMPLEMENTATION", LaunchConfiguration("rmw_implementation")
            ),
            SetEnvironmentVariable(
                "GZ_SIM_SYSTEM_PLUGIN_PATH",
                [
                    gz_ros2_control_lib,
                    os.pathsep,
                    EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                [
                    os.path.dirname(xarm_description_share),
                    os.pathsep,
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
