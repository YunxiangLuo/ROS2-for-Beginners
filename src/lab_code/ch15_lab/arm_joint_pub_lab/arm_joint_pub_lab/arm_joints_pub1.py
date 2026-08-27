#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ArmJointsPub1(Node):
    def __init__(self):
        super().__init__('arm_joints_pub1')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_joint_states)

        self.js = JointState()
        self.js.name = [
            "arm_1_joint", "arm_2_joint", "arm_3_joint",
            "arm_4_joint", "arm_5_joint", "arm_6_joint",
            "gripper_1_joint", "gripper_2_joint"
        ]
        self.js.position = [0.0] * 8
        self.forward = True

        self.get_logger().info('Using urdf with robot_state_publisher ...')

    def publish_joint_states(self):
        self.js.header.stamp = self.get_clock().now().to_msg()

        if self.forward and self.js.position[1] <= 1.5:
            self.js.position[1] += 0.015
        elif self.js.position[1] >= -1.5:
            self.forward = False
            self.js.position[1] -= 0.015
        else:
            self.forward = True

        self.pub.publish(self.js)


def main(args=None):
    rclpy.init(args=args)
    node = ArmJointsPub1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
