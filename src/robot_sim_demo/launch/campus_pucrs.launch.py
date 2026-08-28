"""Launch Wheeltec at the yellow spawn marker in the Campus PUCRS world."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


WORLD_NAME = "campus_pucrs"
SPAWN_X = "20.0"
SPAWN_Y = "0.0"
SPAWN_Z = "0.017"
SPAWN_YAW = "0.0"


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("robot_sim_demo"))
    common_launch = share / "launch" / "gazebo2.launch.py"
    campus_world = share / "worlds" / "campus_pucrs.world.sdf"
    campus_gui_config = share / "gui" / "campus_pucrs.gui.config"
    campus_rviz_config = share / "rviz" / "campus_pucrs.rviz"

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    spawn_robot = LaunchConfiguration("spawn_robot")
    drive = LaunchConfiguration("drive")
    drive_linear_speed = LaunchConfiguration("drive_linear_speed")
    drive_angular_speed = LaunchConfiguration("drive_angular_speed")
    drive_loop = LaunchConfiguration("drive_loop")
    drive_duration = LaunchConfiguration("drive_duration")
    gz_partition = LaunchConfiguration("gz_partition")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui_config = LaunchConfiguration("gui_config")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("spawn_robot", default_value="true"),
            # Keep the robot centered on the marker until the operator commands it.
            DeclareLaunchArgument("drive", default_value="false"),
            DeclareLaunchArgument("drive_linear_speed", default_value="0.18"),
            DeclareLaunchArgument("drive_angular_speed", default_value="0.55"),
            DeclareLaunchArgument("drive_loop", default_value="true"),
            DeclareLaunchArgument("drive_duration", default_value="0.0"),
            DeclareLaunchArgument("gz_partition", default_value="robot_sim_demo"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui_config", default_value=str(campus_gui_config)),
            DeclareLaunchArgument("rviz_config", default_value=str(campus_rviz_config)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(common_launch)),
                launch_arguments={
                    "gui": gui,
                    "rviz": rviz,
                    "spawn_robot": spawn_robot,
                    "drive": drive,
                    "drive_linear_speed": drive_linear_speed,
                    "drive_angular_speed": drive_angular_speed,
                    "drive_loop": drive_loop,
                    "drive_duration": drive_duration,
                    "gz_partition": gz_partition,
                    "use_sim_time": use_sim_time,
                    "world": str(campus_world),
                    "world_name": WORLD_NAME,
                    "gui_config": gui_config,
                    "rviz_config": rviz_config,
                    "spawn_x": SPAWN_X,
                    "spawn_y": SPAWN_Y,
                    "spawn_z": SPAWN_Z,
                    "spawn_yaw": SPAWN_YAW,
                }.items(),
            ),
        ]
    )
