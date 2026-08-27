"""SLAM 建图一键启动 — robot_sim_demo (Gazebo) + slam_sim_demo_ros2 (slam_toolbox).

移动机器人仿真统一使用 `robot_sim_demo/gazebo2.launch.py`（Gazebo Sim
Harmonic + Wheeltec），SLAM 使用 `slam_sim_demo_ros2/slam_demo.launch.py`。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='启动 Gazebo GUI'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='启动 RViz'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('robot_sim_demo'),
                             'launch', 'gazebo2.launch.py'),
            ]),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'rviz': 'false',
                'drive': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('slam_sim_demo_ros2'),
                             'launch', 'slam_demo.launch.py'),
            ]),
            launch_arguments={
                'use_gazebo': 'false',
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_sim_time': 'true',
            }.items(),
        ),
    ])
