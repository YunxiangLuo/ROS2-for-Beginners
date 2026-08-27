"""Pickup and Place action example for MoveIt 2."""

from copy import deepcopy
from math import pi
import time

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from moveit_msgs.action import Pickup, Place
from moveit_msgs.msg import (
    CollisionObject,
    Grasp,
    GripperTranslation,
    MoveItErrorCodes,
    PlaceLocation,
)
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .moveit2 import plan_and_execute, quaternion_from_euler, set_named_goal


class MoveItPickPlaceDemo(Node):
    ARM_GROUP = "xarm"
    GRIPPER_GROUP = "gripper"
    GRIPPER_FRAME = "gripper_centor_link"
    REFERENCE_FRAME = "base_link"
    GRIPPER_JOINT_NAMES = ["gripper_1_joint", "gripper_2_joint"]
    GRIPPER_OPEN = [0.65, 0.65]
    GRIPPER_GRASP = [0.1, 0.1]

    def __init__(self):
        super().__init__("moveit_pick_place_demo")
        self.moveit = MoveItPy(node_name="moveit_pick_place_demo_moveit")
        self.monitor = self.moveit.get_planning_scene_monitor()
        self.arm = self.moveit.get_planning_component(self.ARM_GROUP)
        self.pickup_client = ActionClient(self, Pickup, "/pickup")
        self.place_client = ActionClient(self, Place, "/place")

    def run(self):
        table_id = "table"
        target_id = "target"
        table_pose = self._pose(0.0, 0.0, -0.005)
        target_pose = self._pose(0.47, 0.0, 0.115)
        self._add_box(table_id, table_pose, [1.0, 1.2, 0.01])
        self._add_box(target_id, target_pose, [0.05, 0.05, 0.23])
        time.sleep(1.0)

        set_named_goal(self.arm, "Home")
        if not plan_and_execute(self.moveit, self.arm):
            raise RuntimeError("Failed to plan Home before pickup")

        pickup_goal = Pickup.Goal()
        pickup_goal.target_name = target_id
        pickup_goal.group_name = self.ARM_GROUP
        pickup_goal.end_effector = self.GRIPPER_GROUP
        pickup_goal.possible_grasps = [self._make_grasp(target_pose, table_id)]
        pickup_goal.support_surface_name = table_id
        pickup_goal.attached_object_touch_links = ["gripper_1_link", "gripper_2_link"]
        pickup_goal.allowed_planning_time = 10.0
        pickup_goal.planning_options.replan = True
        if self._execute_action(self.pickup_client, pickup_goal) != MoveItErrorCodes.SUCCESS:
            raise RuntimeError("MoveIt Pickup action failed")

        place_pose = self._pose(0.32, -0.32, 0.115)
        quaternion = quaternion_from_euler(0.0, 0.0, -pi / 4)
        (
            place_pose.pose.orientation.x,
            place_pose.pose.orientation.y,
            place_pose.pose.orientation.z,
            place_pose.pose.orientation.w,
        ) = quaternion
        place_goal = Place.Goal()
        place_goal.group_name = self.ARM_GROUP
        place_goal.attached_object_name = target_id
        place_goal.place_locations = self._make_places(place_pose)
        place_goal.support_surface_name = table_id
        place_goal.allowed_planning_time = 10.0
        place_goal.planning_options.replan = True
        if self._execute_action(self.place_client, place_goal) != MoveItErrorCodes.SUCCESS:
            raise RuntimeError("MoveIt Place action failed")

        set_named_goal(self.arm, "Home")
        if not plan_and_execute(self.moveit, self.arm):
            raise RuntimeError("Failed to return Home after placement")
        self._remove_object(target_id)
        self._remove_object(table_id)

    def _execute_action(self, client, goal, timeout_sec=60.0) -> int:
        if not client.wait_for_server(timeout_sec=10.0):
            return MoveItErrorCodes.FAILURE
        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=10.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return MoveItErrorCodes.FAILURE
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            return MoveItErrorCodes.TIMED_OUT
        return wrapped_result.result.error_code.val

    def _make_grasp(self, pose, support_surface):
        grasp = Grasp()
        grasp.id = "top_grasp"
        grasp.grasp_pose = pose
        grasp.pre_grasp_posture = self._gripper_posture(self.GRIPPER_OPEN)
        grasp.grasp_posture = self._gripper_posture(self.GRIPPER_GRASP)
        grasp.pre_grasp_approach = self._translation(0.1, 0.12, [1.0, 0.0, 0.0])
        grasp.post_grasp_retreat = self._translation(0.08, 0.1, [0.0, 0.0, 1.0])
        grasp.allowed_touch_objects = [support_surface]
        return grasp

    def _make_places(self, pose):
        template = PlaceLocation()
        template.place_pose = pose
        template.pre_place_approach = self._translation(0.08, 0.1, [0.0, 0.0, -1.0])
        template.post_place_retreat = self._translation(0.12, 0.15, [0.0, 0.0, 1.0])
        template.post_place_posture = self._gripper_posture(self.GRIPPER_OPEN)
        places = []
        for x_offset in (0.0, 0.005, 0.01, -0.005, -0.01):
            for y_offset in (0.0, 0.005, 0.01, -0.005, -0.01):
                location = deepcopy(template)
                location.place_pose.pose.position.x += x_offset
                location.place_pose.pose.position.y += y_offset
                places.append(location)
        return places

    def _gripper_posture(self, positions):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.GRIPPER_JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = positions
        point.effort = [1.0, 1.0]
        point.time_from_start = Duration(seconds=5.0).to_msg()
        trajectory.points = [point]
        return trajectory

    def _translation(self, minimum, desired, vector):
        translation = GripperTranslation()
        translation.direction.header.frame_id = self.REFERENCE_FRAME
        translation.direction.vector.x = vector[0]
        translation.direction.vector.y = vector[1]
        translation.direction.vector.z = vector[2]
        translation.min_distance = minimum
        translation.desired_distance = desired
        return translation

    def _pose(self, x: float, y: float, z: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.REFERENCE_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        return pose

    def _add_box(self, object_id, pose, size):
        collision = CollisionObject()
        collision.id = object_id
        collision.header = pose.header
        collision.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = size
        collision.primitives = [primitive]
        collision.primitive_poses = [pose.pose]
        self.monitor.process_collision_object(collision)

    def _remove_object(self, object_id):
        collision = CollisionObject()
        collision.id = object_id
        collision.operation = CollisionObject.REMOVE
        self.monitor.process_collision_object(collision)

    def destroy_node(self):
        self.pickup_client.destroy()
        self.place_client.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MoveItPickPlaceDemo()
    try:
        node.run()
    finally:
        node.moveit.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
