#!/usr/bin/env python3
"""Completed FK/IK exercise using supported MoveItPy goal-state APIs."""

from math import pi
import time

from course_lab_utils.moveit2 import (
    plan_and_execute,
    quaternion_from_euler,
    set_joint_goal,
    set_named_goal,
    set_pose_goal,
)
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
import rclpy
from rclpy.node import Node


class MoveItIkFKDemo(Node):
    ARM_GROUP = "xarm"
    END_EFFECTOR_LINK = "gripper_centor_link"

    def __init__(self):
        super().__init__("moveit_fk_ik_exercise")
        self.moveit = MoveItPy(node_name="moveit_fk_ik_exercise_moveit")
        self.arm = self.moveit.get_planning_component(self.ARM_GROUP)

    def run(self):
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = 0.3
        target.pose.position.y = -0.3
        target.pose.position.z = 0.3
        quaternion = quaternion_from_euler(0.0, 0.0, -pi / 4)
        (
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        ) = quaternion
        set_pose_goal(self.arm, target, self.END_EFFECTOR_LINK)
        self._execute("pose goal")

        positions = [-0.9, -1.0, 0.2, 0.9, -0.76, 1.5]
        set_joint_goal(self.moveit, self.arm, self.ARM_GROUP, positions)
        self._execute("joint goal")

        set_named_goal(self.arm, "Home")
        self._execute("Home")

    def _execute(self, description: str):
        if not plan_and_execute(self.moveit, self.arm, self):
            raise RuntimeError(f"Planning failed for {description}")
        time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = MoveItIkFKDemo()
    try:
        node.run()
    finally:
        node.moveit.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
