#!/usr/bin/env python3
"""Print basic statistics for the current slam_toolbox map."""

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class SlamMonitor(Node):
    def __init__(self):
        super().__init__('slam_monitor')

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos,
        )
        self.map_count = 0
        self.get_logger().info('SLAM map monitor started')

    def map_callback(self, msg: OccupancyGrid):
        self.map_count += 1
        occupied = sum(value >= 65 for value in msg.data)
        unknown = sum(value < 0 for value in msg.data)
        total = len(msg.data)
        known = total - unknown
        free = known - occupied
        known_ratio = known / total * 100.0 if total else 0.0

        self.get_logger().info(
            f'#{self.map_count}: {msg.info.width}x{msg.info.height}, '
            f'{msg.info.resolution:.3f} m/cell, '
            f'occupied={occupied}, free={free}, unknown={unknown}, '
            f'known={known_ratio:.1f}%'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SlamMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
