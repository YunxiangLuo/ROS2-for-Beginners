#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo = LaunchConfiguration('use_gazebo')
    gz_headless = LaunchConfiguration('gz_headless')
    use_sim_time = LaunchConfiguration('use_sim_time')

    navigation_launch_path = os.path.join(
        get_package_share_directory('navigation_sim_demo_ros2'),
        'launch',
        'nav2_demo.launch.py',
    )

    navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_gazebo': use_gazebo,
            'gz_headless': gz_headless,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz2',
        ),
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='true',
            description='Whether to start Gazebo Sim',
        ),
        DeclareLaunchArgument(
            'gz_headless',
            default_value='false',
            description='Whether to run Gazebo without its GUI',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Whether nodes use the simulation clock',
        ),
        navigation_stack,
    ])
