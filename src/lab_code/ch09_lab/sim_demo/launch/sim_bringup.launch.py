"""Ch09 Gazebo 仿真启动 — 委托 robot_sim_demo 的 Gazebo Harmonic 入口.

本工作区的移动机器人仿真统一使用 `robot_sim_demo/gazebo2.launch.py`（Gazebo
Sim Harmonic + ros_gz 桥接 + Wheeltec 机器人）。此 launch 不再维护独立的
旧版 Gazebo 入口；它仅转发常用参数，便于教学时直接 `ros2 launch
sim_demo sim_bringup.launch.py`。
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
        DeclareLaunchArgument('rviz', default_value='false',
                              description='启动 RViz2'),
        DeclareLaunchArgument('drive', default_value='true',
                              description='启动自动巡航驱动'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('robot_sim_demo'),
                    'launch', 'gazebo2.launch.py'),
            ]),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
                'drive': LaunchConfiguration('drive'),
            }.items(),
        ),
    ])
