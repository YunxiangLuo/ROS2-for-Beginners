import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg_dir = get_package_share_directory('navigation_lab')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, '..', '..', '..', 'maps',
                                   'lab10_e2e_replayed_map.yaml'),
        description='Path to map YAML file',
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'navigation_lab.yaml'),
        description='Path to Nav2 params YAML file',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time',
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically start lifecycle nodes',
    )

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        use_sim_time_arg,
        autostart_arg,
        nav2_launch,
    ])
