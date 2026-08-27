#!/usr/bin/env python3
"""Pose-goal inverse-kinematics planning with MoveItPy."""

from math import pi
import time

from course_lab_utils.moveit2 import (
    plan_and_execute,
    quaternion_from_euler,
    set_named_goal,
    set_pose_goal,
)
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
import rclpy
from rclpy.node import Node


class MoveItIKDemo(Node):
    ARM_GROUP = "xarm"
    END_EFFECTOR_LINK = "gripper_centor_link"
    REFERENCE_FRAME = "base_link"

    def __init__(self):
        super().__init__("moveit_ik_demo")
        self.moveit = MoveItPy(node_name="moveit_ik_demo_moveit")
        self.arm = self.moveit.get_planning_component(self.ARM_GROUP)

    def run(self):
        set_named_goal(self.arm, "Home")
        self._execute("Home")

        target = PoseStamped()
        target.header.frame_id = self.REFERENCE_FRAME
        target.pose.position.x = 0.3
        target.pose.position.y = 0.1
        target.pose.position.z = 0.25
        quaternion = quaternion_from_euler(0.0, pi / 2, 0.0)
        (
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        ) = quaternion

        for x_position in (0.3, 0.35):
            target.header.stamp = self.get_clock().now().to_msg()
            target.pose.position.x = x_position
            set_pose_goal(self.arm, target, self.END_EFFECTOR_LINK)
            self._execute(f"pose x={x_position:.2f}")

        set_named_goal(self.arm, "Home")
        self._execute("Home")
        self.get_logger().info("IK demo complete")

    def _execute(self, description: str):
        if not plan_and_execute(self.moveit, self.arm):
            raise RuntimeError(f"Planning failed for {description}")
        time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = MoveItIKDemo()
    try:
        node.run()
    finally:
        node.moveit.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
