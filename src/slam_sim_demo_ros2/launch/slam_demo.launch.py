import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    slam_pkg_share = get_package_share_directory("slam_sim_demo_ros2")
    slam_toolbox_share = get_package_share_directory("slam_toolbox")
    robot_sim_share = get_package_share_directory("robot_sim_demo")

    default_params = os.path.join(slam_pkg_share, "params", "slam_toolbox_params.yaml")
    default_rviz = os.path.join(slam_pkg_share, "rviz", "slam.rviz")
    default_world = os.path.join(robot_sim_share, "worlds", "museum.sdf")
    robot_sim_launch = os.path.join(robot_sim_share, "launch", "gazebo2.launch.py")
    slam_toolbox_launch = os.path.join(slam_toolbox_share, "launch", "online_async_launch.py")

    slam_params_file = LaunchConfiguration("slam_params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo = LaunchConfiguration("use_gazebo")
    gz_headless = LaunchConfiguration("gz_headless")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_topic = LaunchConfiguration("scan_topic")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    configured_slam_params = RewrittenYaml(
        source_file=slam_params_file,
        root_key="",
        param_rewrites={"scan_topic": scan_topic},
        convert_types=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("slam_params_file", default_value=default_params),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.03"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_gazebo", default_value="false"),
            DeclareLaunchArgument("gz_headless", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_sim_launch),
                condition=IfCondition(use_gazebo),
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
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_toolbox_launch),
                launch_arguments={
                    "slam_params_file": configured_slam_params,
                    "use_sim_time": use_sim_time,
                }.items(),
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
