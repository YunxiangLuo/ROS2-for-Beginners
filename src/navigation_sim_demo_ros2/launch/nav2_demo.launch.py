import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.descriptions import ParameterFile
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    nav_pkg_share = get_package_share_directory("navigation_sim_demo_ros2")
    robot_sim_share = get_package_share_directory("robot_sim_demo")

    default_map = os.path.join(nav_pkg_share, "maps", "Software_Museum.yaml")
    default_params = os.path.join(nav_pkg_share, "params", "nav2_params.yaml")
    default_rviz = os.path.join(nav_pkg_share, "rviz", "navigation.rviz")
    default_world = os.path.join(robot_sim_share, "worlds", "museum.sdf")
    robot_sim_launch = os.path.join(robot_sim_share, "launch", "gazebo2.launch.py")

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo = LaunchConfiguration("use_gazebo")
    gz_headless = LaunchConfiguration("gz_headless")
    world = LaunchConfiguration("world")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_delay_sec = LaunchConfiguration("initial_pose_delay_sec")
    use_respawn = LaunchConfiguration("use_respawn")
    use_sim_time = LaunchConfiguration("use_sim_time")
    lifecycle_delay_sec = LaunchConfiguration("lifecycle_delay_sec")
    log_level = LaunchConfiguration("log_level")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml}
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.03"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_gazebo", default_value="false"),
            DeclareLaunchArgument("gz_headless", default_value="true"),
            DeclareLaunchArgument("use_respawn", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("lifecycle_delay_sec", default_value="1.5"),
            DeclareLaunchArgument("initial_pose_delay_sec", default_value="4.0"),
            DeclareLaunchArgument("log_level", default_value="info"),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(robot_sim_launch),
                        launch_arguments={
                            "gui": PythonExpression(
                                ["'false' if '", gz_headless, "' == 'true' else 'true'"]
                            ),
                            "rviz": "false",
                            "world": world,
                            "spawn_robot": "true",
                            "drive": "false",
                            "spawn_x": spawn_x,
                            "spawn_y": spawn_y,
                            "spawn_z": spawn_z,
                            "spawn_yaw": spawn_yaw,
                            "use_sim_time": use_sim_time,
                        }.items(),
                    )
                ],
                condition=IfCondition(use_gazebo),
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings
                + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "/cmd_vel")],
            ),
            TimerAction(
                period=lifecycle_delay_sec,
                actions=[
                    Node(
                        package="navigation_sim_demo_ros2",
                        executable="nav2_lifecycle_runner",
                        name="nav2_lifecycle_runner",
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=initial_pose_delay_sec,
                actions=[
                    Node(
                        package="navigation_sim_demo_ros2",
                        executable="initial_pose_publisher",
                        name="initial_pose_publisher",
                        output="screen",
                        parameters=[
                            {
                                "x": initial_pose_x,
                                "y": initial_pose_y,
                                "yaw": initial_pose_yaw,
                                "use_sim_time": use_sim_time,
                            }
                        ],
                    ),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
