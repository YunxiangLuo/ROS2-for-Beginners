import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('urdf_demo')
    urdf_path = os.path.join(pkg_path, 'urdf', 'simple_robot.xacro')
    rviz_path = os.path.join(pkg_path, 'rviz', 'display.rviz')

    use_gui = LaunchConfiguration('gui', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 joint_state_publisher_gui'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
            output='screen',
        ),
    ])
