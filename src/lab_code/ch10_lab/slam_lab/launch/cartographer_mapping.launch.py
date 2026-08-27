#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('slam_lab')

    configuration_directory = LaunchConfiguration('configuration_directory')
    configuration_basename = LaunchConfiguration('configuration_basename')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    resolution = LaunchConfiguration('resolution')
    use_sim_time = LaunchConfiguration('use_sim_time')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory',
            configuration_directory,
            '-configuration_basename',
            configuration_basename,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', scan_topic),
            ('odom', odom_topic),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=[
            '-resolution',
            resolution,
            '-publish_period_sec',
            '1.0',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'configuration_directory',
            default_value=os.path.join(
                package_share,
                'config',
                'cartographer',
            ),
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='xbot_2d.lua',
        ),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        cartographer_node,
        occupancy_grid_node,
    ])
