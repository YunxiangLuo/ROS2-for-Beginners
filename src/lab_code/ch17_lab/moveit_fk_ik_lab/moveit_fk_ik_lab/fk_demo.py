#!/usr/bin/env python3
"""Joint-space motion planning with the ROS 2 Jazzy MoveItPy API."""

import time

from course_lab_utils.moveit2 import plan_and_execute, set_joint_goal, set_named_goal
from moveit.planning import MoveItPy
import rclpy
from rclpy.node import Node


class MoveItFKDemo(Node):
    ARM_GROUP = "xarm"
    GRIPPER_GROUP = "gripper"

    def __init__(self):
        super().__init__("moveit_fk_demo")
        self.moveit = MoveItPy(node_name="moveit_fk_demo_moveit")
        self.arm = self.moveit.get_planning_component(self.ARM_GROUP)
        self.gripper = self.moveit.get_planning_component(self.GRIPPER_GROUP)

    def run(self):
        for positions in (
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [-0.664, -0.775, 0.675, -1.241, -0.473, -1.281],
        ):
            set_joint_goal(self.moveit, self.arm, self.ARM_GROUP, positions)
            self._execute(self.arm, f"arm joints: {positions}")

        for positions in ([0.65, 0.65], [0.0, 0.0]):
            set_joint_goal(self.moveit, self.gripper, self.GRIPPER_GROUP, positions)
            self._execute(self.gripper, f"gripper joints: {positions}")

        set_named_goal(self.arm, "Home")
        self._execute(self.arm, "Home")
        self.get_logger().info("FK demo complete")

    def _execute(self, component, description: str):
        if not plan_and_execute(self.moveit, component):
            raise RuntimeError(f"Planning failed for {description}")
        self.get_logger().info(description)
        time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = MoveItFKDemo()
    try:
        node.run()
    finally:
        node.moveit.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
