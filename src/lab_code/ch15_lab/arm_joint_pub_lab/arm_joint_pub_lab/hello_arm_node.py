#!/usr/bin/env python3
"""
hello_arm_node.py — ROS2节点示例
发布虚拟机械臂关节状态并打印日志
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class HelloArmNode(Node):
    def __init__(self):
        super().__init__('hello_arm')

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        self.js = JointState()
        self.js.name = [
            'joint1', 'joint2', 'joint3',
            'finger1_joint', 'finger2_joint'
        ]
        self.js.position = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.joint2_angle = 0.0
        self.joint3_angle = 0.0
        self.finger_open = 0.0
        self.forward = True

        self.get_logger().info('HelloArmNode 已启动 — 欢迎来到ROS2机械臂编程世界！')

    def publish_joint_states(self):
        self.js.header.stamp = self.get_clock().now().to_msg()

        self.joint2_angle += 0.02 if self.forward else -0.02
        if self.joint2_angle > 1.5:
            self.forward = False
        elif self.joint2_angle < -1.5:
            self.forward = True

        self.joint3_angle = self.joint2_angle * 0.5
        self.finger_open = 0.02 if self.forward else -0.02

        self.js.position = [0.0, self.joint2_angle, self.joint3_angle,
                            self.finger_open, -self.finger_open]

        self.pub.publish(self.js)
        self.get_logger().info(
            f'关节角度 — joint1: {self.js.position[0]:.3f}, '
            f'joint2: {self.js.position[1]:.3f}, '
            f'joint3: {self.js.position[2]:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = HelloArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
