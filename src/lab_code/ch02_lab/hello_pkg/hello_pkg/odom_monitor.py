#!/usr/bin/env python3
"""odom_monitor: 监听 XBot-U /odom 话题，实时显示机器人位置。"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomMonitor(Node):
    @staticmethod
    def quaternion_to_yaw(orientation):
        """Convert a unit quaternion to yaw using the complete quaternion."""
        return math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )

    def __init__(self):
        super().__init__('odom_monitor')
        self.sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(orient)
        self.get_logger().info(
            f'XBot-U 位置: x={pos.x:.2f}m, y={pos.y:.2f}m, 航向={yaw:.2f}rad')


def main(args=None):
    rclpy.init(args=args)
    node = OdomMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
