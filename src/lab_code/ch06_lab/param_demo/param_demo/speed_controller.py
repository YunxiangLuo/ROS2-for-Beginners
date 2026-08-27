#!/usr/bin/env python3
"""dynamic_speed: 动态参数调速 — 通过 ros2 param set 实时修改机器人速度"""
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist


class SpeedController(Node):
    def __init__(self):
        super().__init__('speed_controller')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.0)
        self.declare_parameter('enable_control', True)
        self.add_on_set_parameters_callback(self.validate)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def validate(self, params):
        for p in params:
            if p.name == 'linear_speed' and abs(p.value) > 1.0:
                return SetParametersResult(
                    successful=False, reason='线速度不能超过 ±1.0m/s')
            if p.name == 'angular_speed' and abs(p.value) > 2.0:
                return SetParametersResult(
                    successful=False, reason='角速度不能超过 ±2.0rad/s')
        return SetParametersResult(successful=True)

    def control_loop(self):
        if not self.get_parameter('enable_control').value:
            return
        msg = Twist()
        msg.linear.x = self.get_parameter('linear_speed').value
        msg.angular.z = self.get_parameter('angular_speed').value
        self.pub.publish(msg)
        self.get_logger().info(
            f'速度: v={msg.linear.x:.1f}, ω={msg.angular.z:.1f}',
            throttle_duration_sec=2)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SpeedController())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
