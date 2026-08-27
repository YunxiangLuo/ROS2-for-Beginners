import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('arm_description')
    xacro_path = os.path.join(pkg_path, 'urdf', 'three_dof_arm.xacro')

    use_gui = LaunchConfiguration('gui', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='是否启动 joint_state_publisher_gui'
        ),

        # robot_state_publisher: 加载URDF并发布TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', xacro_path])}],
        ),

        # joint_state_publisher_gui: 滑块控制关节
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
        ),

        # RViz2 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
