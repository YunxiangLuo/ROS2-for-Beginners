import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    slam_pkg_share = get_package_share_directory("slam_sim_demo_ros2")
    slam_launch = os.path.join(slam_pkg_share, "launch", "slam_demo.launch.py")
    default_params = os.path.join(slam_pkg_share, "params", "slam_toolbox_params.yaml")
    default_rviz = os.path.join(slam_pkg_share, "rviz", "slam.rviz")
    robot_sim_share = get_package_share_directory("robot_sim_demo")
    default_world = os.path.join(robot_sim_share, "worlds", "museum.sdf")

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
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    "slam_params_file": slam_params_file,
                    "rviz_config": rviz_config,
                    "use_rviz": use_rviz,
                    "use_gazebo": use_gazebo,
                    "gz_headless": gz_headless,
                    "world": world,
                    "use_sim_time": use_sim_time,
                    "scan_topic": scan_topic,
                    "spawn_x": spawn_x,
                    "spawn_y": spawn_y,
                    "spawn_z": spawn_z,
                    "spawn_yaw": spawn_yaw,
                }.items(),
            ),
        ]
    )
