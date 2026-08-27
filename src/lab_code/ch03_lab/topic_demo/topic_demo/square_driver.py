#!/usr/bin/env python3
"""twist_square: 发布 Twist 控制 XBot-U 走 1m×1m 正方形轨迹"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def make_twist(linear, angular):
    """Create a planar velocity command for the square trajectory."""
    msg = Twist()
    msg.linear.x = float(linear)
    msg.angular.z = float(angular)
    return msg


class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('方形轨迹控制器就绪 — 5秒后开始')
        time.sleep(5)
        self.drive_square()

    def move(self, linear, angular, duration):
        msg = make_twist(linear, angular)
        end_time = time.time() + duration
        while time.time() < end_time and rclpy.ok():
            self.pub.publish(msg)
            time.sleep(0.1)

    def stop(self):
        self.move(0.0, 0.0, 0.5)

    def drive_square(self):
        self.get_logger().info('开始走正方形轨迹...')
        for i in range(4):
            self.get_logger().info(f'边 {i+1}: 直行')
            self.move(0.2, 0.0, 5.0)
            self.get_logger().info(f'边 {i+1}: 左转90°')
            self.move(0.0, 1.57, 1.0)
        self.stop()
        self.get_logger().info('正方形轨迹完成！')


def main(args=None):
    rclpy.init(args=args)
    try:
        SquareDriver()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
