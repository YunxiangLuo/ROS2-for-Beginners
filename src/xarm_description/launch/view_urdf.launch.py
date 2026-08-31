from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    display_launch = PathJoinSubstitution(
        [FindPackageShare("xarm_description"), "launch", "display.launch.py"]
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(display_launch),
                launch_arguments={"use_gui": "false"}.items(),
            )
        ]
    )
