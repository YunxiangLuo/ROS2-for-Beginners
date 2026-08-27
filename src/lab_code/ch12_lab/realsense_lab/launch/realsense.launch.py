"""RealSense D400 相机启动 — 委托 realsense2_camera 的 rs_launch.py.

参数:
    pointcloud: 是否启用点云与深度对齐 (默认 true)
    camera_name: 相机节点命名空间 (默认 camera)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    realsense_share = get_package_share_directory('realsense2_camera')
    rs_launch = os.path.join(realsense_share, 'launch', 'rs_launch.py')

    pointcloud = LaunchConfiguration('pointcloud')

    return LaunchDescription([
        DeclareLaunchArgument(
            'pointcloud', default_value='true',
            description='启用点云与深度对齐'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rs_launch]),
            launch_arguments={
                'depth_module.depth_profile': '640x480x30',
                'rgb_camera.color_profile': '640x480x30',
                'pointcloud.enable': pointcloud,
                'align_depth.enable': pointcloud,
                'enable_sync': 'true',
            }.items(),
        ),
    ])
