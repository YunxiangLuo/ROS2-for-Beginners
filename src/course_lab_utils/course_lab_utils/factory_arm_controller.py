"""Pick, transfer and pour sequence with explicit simulation mode."""

from copy import deepcopy
import time

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
import rclpy
from rclpy.node import Node

from .moveit2 import plan_and_execute, set_joint_goal, set_pose_goal


class ArmController(Node):
    ARM_GROUP = "xarm"
    GRIPPER_GROUP = "gripper"
    END_EFFECTOR_LINK = "gripper_centor_link"

    def __init__(self):
        super().__init__("arm_controller")
        self.declare_parameter("simulation_mode", True)
        self.simulation_mode = bool(self.get_parameter("simulation_mode").value)
        self.moveit = None
        self.arm = None
        self.gripper = None
        if not self.simulation_mode:
            self.moveit = MoveItPy(node_name="factory_arm_controller_moveit")
            self.arm = self.moveit.get_planning_component(self.ARM_GROUP)
            self.gripper = self.moveit.get_planning_component(self.GRIPPER_GROUP)

    def execute_pick_and_place(self, target_pose, material_name):
        pre_grasp = deepcopy(target_pose)
        pre_grasp.pose.position.z += 0.1
        self.plan_move(pre_grasp, f"move above {material_name}")
        self.plan_move(target_pose, f"grasp {material_name}")
        self.close_gripper()

        pour_pose = PoseStamped()
        pour_pose.header.frame_id = target_pose.header.frame_id or "base_link"
        pour_pose.pose.position.x = 0.5
        pour_pose.pose.position.z = 0.4
        pour_pose.pose.orientation.w = 1.0
        self.plan_move(pour_pose, "transfer above the test tube")
        self.pour(duration_sec=2.0)
        self.open_gripper()

    def plan_move(self, pose, description):
        self.get_logger().info(f"Plan: {description}")
        if self.simulation_mode:
            return True
        set_pose_goal(self.arm, pose, self.END_EFFECTOR_LINK)
        if not plan_and_execute(self.moveit, self.arm, self):
            raise RuntimeError(f"Planning failed: {description}")
        return True

    def close_gripper(self):
        self._move_gripper([0.0, 0.0])

    def open_gripper(self):
        self._move_gripper([0.65, 0.65])

    def _move_gripper(self, positions):
        if self.simulation_mode:
            return True
        set_joint_goal(
            self.moveit,
            self.gripper,
            self.GRIPPER_GROUP,
            positions,
        )
        if not plan_and_execute(self.moveit, self.gripper, self):
            raise RuntimeError("Gripper planning failed")
        return True

    def pour(self, duration_sec):
        if not self.simulation_mode:
            time.sleep(duration_sec)

    def destroy_node(self):
        if self.moveit is not None:
            self.moveit.shutdown()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
