"""启动 talker + listener 的 Launch 文件"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_py', executable='talker',
            name='my_talker', output='screen'),
        Node(
            package='demo_nodes_py', executable='listener',
            name='my_listener', output='screen'),
    ])
