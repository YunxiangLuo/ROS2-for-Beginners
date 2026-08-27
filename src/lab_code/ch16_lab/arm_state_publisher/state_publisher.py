#!/usr/bin/env python3
"""
state_publisher.py — ROS2节点
向 /joint_states 话题发布三自由度机械臂关节角度
配合 robot_state_publisher 使用，发布TF变换
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ArmStatePublisher(Node):
    def __init__(self):
        super().__init__('arm_state_publisher')

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_joint_states)

        self.js = JointState()
        self.js.name = [
            'joint1', 'joint2', 'joint3',
            'finger1_joint', 'finger2_joint'
        ]
        self.js.position = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.forward = True
        self.time = 0.0

        self.get_logger().info('ArmStatePublisher 已启动，正在发布关节状态...')

    def publish_joint_states(self):
        self.js.header.stamp = self.get_clock().now().to_msg()

        self.time += 0.05

        # joint1 缓慢旋转 (正弦轨迹)
        self.js.position[0] = 1.0 * self._sin(self.time * 0.5)

        # joint2 往复摆动
        if self.forward:
            self.js.position[1] += 0.02
            if self.js.position[1] >= 1.5:
                self.forward = False
        else:
            self.js.position[1] -= 0.02
            if self.js.position[1] <= -1.5:
                self.forward = True

        # joint3 跟随 joint2
        self.js.position[2] = self.js.position[1] * 0.6

        # 夹爪周期性开合
        finger_pos = 0.015 * self._sin(self.time * 1.5)
        self.js.position[3] = finger_pos
        self.js.position[4] = -finger_pos

        self.pub.publish(self.js)

    def _sin(self, x):
        import math
        return math.sin(x)


def main(args=None):
    rclpy.init(args=args)
    node = ArmStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ArmStatePublisher 已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
