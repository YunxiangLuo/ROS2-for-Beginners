#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ArmGripper(Node):
    def __init__(self):
        super().__init__('arm_gripper')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_joint_states)

        self.js = JointState()
        self.js.name = [
            "arm_1_joint", "arm_2_joint", "arm_3_joint",
            "arm_4_joint", "arm_5_joint", "arm_6_joint",
            "gripper_1_joint", "gripper_2_joint"
        ]
        self.js.position = [0.0] * 8
        self.cycle = 0

        self.get_logger().info('Using urdf with robot_state_publisher ...')

    def publish_joint_states(self):
        self.js.header.stamp = self.get_clock().now().to_msg()

        if self.cycle < 100:
            self.js.position[0] += 0.02
            self.js.position[3] -= 0.015
            self.js.position[6] += 0.0065
            self.js.position[7] += 0.0065
        elif self.cycle < 200:
            self.js.position[0] -= 0.02
            self.js.position[3] += 0.015
            self.js.position[6] -= 0.0065
            self.js.position[7] -= 0.0065
        else:
            self.cycle = 0

        self.cycle += 1
        self.pub.publish(self.js)


def main(args=None):
    rclpy.init(args=args)
    node = ArmGripper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
