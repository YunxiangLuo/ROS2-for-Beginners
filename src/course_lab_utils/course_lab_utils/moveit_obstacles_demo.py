"""Planning-scene obstacle example for MoveItPy."""

from math import pi
import time

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from .moveit2 import (
    plan_and_execute,
    quaternion_from_euler,
    scene_object_ids,
    set_named_goal,
    set_pose_goal,
)


class MoveItObstaclesDemo(Node):
    ARM_GROUP = "xarm"
    END_EFFECTOR_LINK = "gripper_centor_link"
    REFERENCE_FRAME = "base_link"

    def __init__(self):
        super().__init__("moveit_obstacles_demo")
        self.moveit = MoveItPy(node_name="moveit_obstacles_demo_moveit")
        self.monitor = self.moveit.get_planning_scene_monitor()
        self.arm = self.moveit.get_planning_component(self.ARM_GROUP)
        self.scene_publisher = self.create_publisher(PlanningScene, "/planning_scene", 10)

    def run(self):
        set_named_goal(self.arm, "Home")
        self._execute("Home")

        objects = (
            ("table", self._pose(0.0, 0.0, -0.005), SolidPrimitive.BOX, [1.0, 1.2, 0.01]),
            ("sphere", self._pose(0.3, 0.2, 0.12), SolidPrimitive.SPHERE, [0.12]),
            ("box", self._pose(0.3, -0.2, 0.2), SolidPrimitive.BOX, [0.25, 0.05, 0.4]),
        )
        for object_id, pose, primitive_type, dimensions in objects:
            input(f"Press Enter to add {object_id}...")
            self._add_primitive(object_id, pose, primitive_type, dimensions)
            if not self._wait_for_object(object_id):
                raise RuntimeError(f"Planning scene did not add {object_id}")

        self._publish_colors()
        input("Press Enter to plan around the obstacles...")
        target = self._pose(0.25, -0.4, 0.25)
        quaternion = quaternion_from_euler(0.0, 0.0, -pi / 4)
        (
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        ) = quaternion
        set_pose_goal(self.arm, target, self.END_EFFECTOR_LINK)
        self._execute("obstacle-aware pose")

        set_named_goal(self.arm, "Home")
        self._execute("Home")
        input("Press Enter to remove the obstacles...")
        for object_id, *_ in objects:
            self._remove_object(object_id)

    def _pose(self, x: float, y: float, z: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.REFERENCE_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        return pose

    def _add_primitive(self, object_id, pose, primitive_type, dimensions):
        collision = CollisionObject()
        collision.id = object_id
        collision.header = pose.header
        collision.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = primitive_type
        primitive.dimensions = dimensions
        collision.primitives = [primitive]
        collision.primitive_poses = [pose.pose]
        self.monitor.process_collision_object(collision)

    def _remove_object(self, object_id: str):
        collision = CollisionObject()
        collision.id = object_id
        collision.operation = CollisionObject.REMOVE
        self.monitor.process_collision_object(collision)

    def _wait_for_object(self, object_id: str, timeout_sec: float = 4.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if object_id in scene_object_ids(self.monitor):
                return True
            time.sleep(0.1)
        return False

    def _publish_colors(self):
        scene = PlanningScene()
        scene.is_diff = True
        for object_id, red, green, blue in (
            ("box", 0.8, 0.8, 0.0),
            ("sphere", 0.8, 0.0, 0.9),
        ):
            color = ObjectColor()
            color.id = object_id
            color.color.r = red
            color.color.g = green
            color.color.b = blue
            color.color.a = 1.0
            scene.object_colors.append(color)
        self.scene_publisher.publish(scene)

    def _execute(self, description: str):
        if not plan_and_execute(self.moveit, self.arm, self):
            raise RuntimeError(f"Planning failed for {description}")
        time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = MoveItObstaclesDemo()
    try:
        node.run()
    finally:
        node.moveit.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
