"""Attach and detach collision objects with MoveItPy."""

from math import pi
import time

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from .moveit2 import (
    attached_object_ids,
    plan_and_execute,
    quaternion_from_euler,
    scene_object_ids,
    set_named_goal,
    set_pose_goal,
)


class MoveItAttachDemo(Node):
    ARM_GROUP = "xarm"
    END_EFFECTOR_LINK = "gripper_centor_link"
    REFERENCE_FRAME = "base_link"

    def __init__(self):
        super().__init__("moveit_attach_demo")
        self.moveit = MoveItPy(node_name="moveit_attach_demo_moveit")
        self.monitor = self.moveit.get_planning_scene_monitor()
        self.arm = self.moveit.get_planning_component(self.ARM_GROUP)

    def run(self):
        set_named_goal(self.arm, "Home")
        self._execute("Home")

        objects = (
            ("table", self._pose(0.0, 0.0, -0.005), SolidPrimitive.BOX, [1.0, 1.2, 0.01]),
            ("sphere", self._pose(0.3, 0.2, 0.12), SolidPrimitive.SPHERE, [0.12]),
            ("box", self._pose(0.3, -0.2, 0.2), SolidPrimitive.BOX, [0.25, 0.05, 0.4]),
        )
        input("Press Enter to add planning-scene objects...")
        for object_id, pose, primitive_type, dimensions in objects:
            self._add_world_object(object_id, pose, primitive_type, dimensions)
            if not self._wait_for_world_object(object_id):
                raise RuntimeError(f"Planning scene did not add {object_id}")

        input("Press Enter to attach the tool...")
        tool_pose = PoseStamped()
        tool_pose.header.frame_id = self.END_EFFECTOR_LINK
        tool_pose.pose.position.x = -0.01
        tool_pose.pose.orientation.w = 1.0
        self._attach_tool("tool", tool_pose, [0.025, 0.025, 0.16])
        if not self._wait_for_attached_object("tool"):
            raise RuntimeError("Planning scene did not attach the tool")

        input("Press Enter to move the arm...")
        target = self._pose(0.25, -0.4, 0.25)
        quaternion = quaternion_from_euler(0.0, 0.0, -pi / 4)
        (
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        ) = quaternion
        set_pose_goal(self.arm, target, self.END_EFFECTOR_LINK)
        self._execute("tool pose")
        for configuration in ("Down", "Home"):
            set_named_goal(self.arm, configuration)
            self._execute(configuration)

        input("Press Enter to detach the tool...")
        self._detach_object("tool")
        input("Press Enter to remove the obstacles...")
        for object_id, *_ in objects:
            self._remove_world_object(object_id)

    def _pose(self, x: float, y: float, z: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.REFERENCE_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        return pose

    def _make_collision(self, object_id, pose, primitive_type, dimensions):
        collision = CollisionObject()
        collision.id = object_id
        collision.header = pose.header
        collision.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = primitive_type
        primitive.dimensions = dimensions
        collision.primitives = [primitive]
        collision.primitive_poses = [pose.pose]
        return collision

    def _add_world_object(self, object_id, pose, primitive_type, dimensions):
        self.monitor.process_collision_object(
            self._make_collision(object_id, pose, primitive_type, dimensions)
        )

    def _attach_tool(self, object_id, pose, dimensions):
        attached = AttachedCollisionObject()
        attached.link_name = self.END_EFFECTOR_LINK
        attached.object = self._make_collision(
            object_id, pose, SolidPrimitive.BOX, dimensions
        )
        attached.touch_links = ["gripper_1_link", "gripper_2_link"]
        self.monitor.process_attached_collision_object(attached)

    def _detach_object(self, object_id: str):
        attached = AttachedCollisionObject()
        attached.link_name = self.END_EFFECTOR_LINK
        attached.object.id = object_id
        attached.object.operation = CollisionObject.REMOVE
        self.monitor.process_attached_collision_object(attached)

    def _remove_world_object(self, object_id: str):
        collision = CollisionObject()
        collision.id = object_id
        collision.operation = CollisionObject.REMOVE
        self.monitor.process_collision_object(collision)

    def _wait_for_world_object(self, object_id, timeout_sec=4.0):
        return self._wait_for_id(scene_object_ids, object_id, timeout_sec)

    def _wait_for_attached_object(self, object_id, timeout_sec=4.0):
        return self._wait_for_id(attached_object_ids, object_id, timeout_sec)

    def _wait_for_id(self, reader, object_id, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if object_id in reader(self.monitor):
                return True
            time.sleep(0.1)
        return False

    def _execute(self, description: str):
        if not plan_and_execute(self.moveit, self.arm, self):
            raise RuntimeError(f"Planning failed for {description}")
        time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = MoveItAttachDemo()
    try:
        node.run()
    finally:
        node.moveit.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
