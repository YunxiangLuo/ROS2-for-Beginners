#!/usr/bin/env python3
"""
check_topics.py — 列出并验证CARLA ROS2 Bridge的话题。

功能：
1. 列出所有以 /carla/ 开头的ROS2话题
2. 验证关键话题（传感器、车辆状态、控制接口）是否存在
3. 检查每个话题的消息类型
4. 可选：输出缺失话题的警告

用法：
    python3 check_topics.py
    python3 check_topics.py --role-name ego_vehicle
    python3 check_topics.py --verbose
"""


# Windows GBK 控制台输出 Unicode 符号(勾/叉)会抛 UnicodeEncodeError, 统一切换到 UTF-8
import sys as _sys
if hasattr(_sys.stdout, 'reconfigure'):
    try:
        _sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy


class CarlaTopicChecker(Node):
    """检查CARLA ROS2 Bridge发布的话题状态"""

    EXPECTED_SENSORS = {
        'gnss': 'nav_msgs/msg/Odometry',
        'imu': 'sensor_msgs/msg/Imu',
        'odometry': 'nav_msgs/msg/Odometry',
        'vehicle_status': 'carla_msgs/msg/CarlaEgoVehicleStatus',
        'vehicle_control_cmd': 'carla_msgs/msg/CarlaEgoVehicleControl',
        'collision': 'carla_msgs/msg/CarlaCollisionEvent',
        'lane_invasion': 'carla_msgs/msg/CarlaLaneInvasionEvent',
    }

    OPTIONAL_TOPICS = {
        'rgb_front/image': 'sensor_msgs/msg/Image',
        'rgb_front/camera_info': 'sensor_msgs/msg/CameraInfo',
        'depth_front/image': 'sensor_msgs/msg/Image',
        'lidar': 'sensor_msgs/msg/PointCloud2',
        'radar_front': 'sensor_msgs/msg/RadarDetectionArray',
    }

    def __init__(self, role_name='ego_vehicle', verbose=False):
        super().__init__('carla_topic_checker')
        self.role_name = role_name
        self.verbose = verbose
        self.prefix = f'/carla/{role_name}/'

    def check_topics(self):
        """检查所有CARLA相关话题"""
        all_topics = self.get_topic_names_and_types()
        carla_topics = [
            (name, types)
            for name, types in all_topics
            if name.startswith('/carla/')
        ]

        ego_topics = [
            (name, types)
            for name, types in carla_topics
            if name.startswith(self.prefix)
        ]

        print('=' * 60)
        print(f'  CARLA ROS2 Topic Checker')
        print(f'  Role name: {self.role_name}')
        print('=' * 60)

        # 列出所有CARLA话题
        print(f'\n[All CARLA Topics ({len(carla_topics)})]')
        print('-' * 60)
        if not carla_topics:
            print('  (none found — Bridge may not be running)')

        for name, types in sorted(carla_topics):
            echo = '←' if name.startswith(self.prefix) else ' '
            type_str = types[0] if types else 'unknown'
            print(f'  {echo} {name}')
            if self.verbose:
                print(f'       Type: {type_str}')

        # 检查Ego话题
        print(f'\n[Ego Vehicle Topics ({self.prefix}*) — {len(ego_topics)}]')
        print('-' * 60)

        missing = []
        ego_topic_names = {n for n, _ in ego_topics}

        for topic_suffix, expected_type in self.EXPECTED_SENSORS.items():
            full_topic = f'{self.prefix}{topic_suffix}'
            if full_topic in ego_topic_names:
                actual_type = dict(ego_topics).get(full_topic, ['?'])[0]
                ok = '✓' if actual_type == expected_type else '✗'
                print(f'  {ok} {full_topic}')
                if self.verbose and ok != '✓':
                    print(f'       Expected: {expected_type}')
                    print(f'       Actual:   {actual_type}')
            else:
                print(f'  ✗ {full_topic}  [MISSING]')
                missing.append(full_topic)

        for topic_suffix, expected_type in self.OPTIONAL_TOPICS.items():
            full_topic = f'{self.prefix}{topic_suffix}'
            if full_topic in ego_topic_names:
                actual_type = dict(ego_topics).get(full_topic, ['?'])[0]
                ok = '✓' if actual_type == expected_type else '✗'
                print(f'  {ok} {full_topic}  (optional)')
                if self.verbose and ok != '✓':
                    print(f'       Expected: {expected_type}')
                    print(f'       Actual:   {actual_type}')

        # 检查Services
        print(f'\n[Known Services]')
        all_services = self.get_service_names_and_types()
        carla_services = [
            n for n, _ in all_services
            if n.startswith(self.prefix)
        ]
        for svc in sorted(carla_services):
            print(f'  ~ {svc}')

        # 汇总
        print(f'\n[Summary]')
        print(f'  Total CARLA topics:     {len(carla_topics)}')
        print(f'  Ego vehicle topics:     {len(ego_topics)}')
        print(f'  Missing required:       {len(missing)}')

        if missing:
            print(f'\n  WARNING: Missing required topics:')
            for t in missing:
                print(f'    - {t}')
            print(f'\n  Tip: Ensure the Ego Vehicle is spawned with '
                  f'role_name="{self.role_name}"')
            return False

        print(f'\n  All required topics are present!')
        return True


def main():
    parser = argparse.ArgumentParser(
        description='Check CARLA ROS2 Bridge topics')
    parser.add_argument('--role-name', default='ego_vehicle',
                        help='Vehicle role name (default: ego_vehicle)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Show detailed topic info')
    args = parser.parse_args()

    rclpy.init()
    checker = CarlaTopicChecker(
        role_name=args.role_name,
        verbose=args.verbose)

    success = checker.check_topics()

    # Spin briefly to allow node to gather info
    rclpy.spin_once(checker, timeout_sec=0.5)

    checker.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
