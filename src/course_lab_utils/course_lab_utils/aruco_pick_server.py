"""Execute Pickup and Place actions for TF-corrected ArUco marker poses."""

from copy import deepcopy
from math import atan2, pi
import time

from course_lab_interfaces.msg import MarkerPoseArray
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
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .moveit2 import plan_and_execute, quaternion_from_euler, set_named_goal


ARM_GROUP = "xarm"
GRIPPER_GROUP = "gripper"
GRIPPER_FRAME = "gripper_centor_link"
REFERENCE_FRAME = "base_link"
GRIPPER_JOINT_NAMES = ["gripper_1_joint", "gripper_2_joint"]
GRIPPER_OPEN = [0.68, 0.68]
GRIPPER_GRASP = [0.25, 0.25]


def make_gripper_posture(positions):
    trajectory = JointTrajectory()
    trajectory.joint_names = GRIPPER_JOINT_NAMES
    point = JointTrajectoryPoint()
    point.positions = positions
    point.effort = [1.0, 1.0]
    point.time_from_start = Duration(seconds=5.0).to_msg()
    trajectory.points = [point]
    return trajectory


def make_translation(minimum, desired, vector):
    translation = GripperTranslation()
    translation.direction.header.frame_id = REFERENCE_FRAME
    translation.direction.vector.x = vector[0]
    translation.direction.vector.y = vector[1]
    translation.direction.vector.z = vector[2]
    translation.min_distance = minimum
    translation.desired_distance = desired
    return translation


def make_grasps(target_pose, target_id, support_surface):
    template = Grasp()
    template.grasp_pose = deepcopy(target_pose)
    template.grasp_pose.pose.position.z += 0.08
    template.pre_grasp_posture = make_gripper_posture(GRIPPER_OPEN)
    template.grasp_posture = make_gripper_posture(GRIPPER_GRASP)
    template.pre_grasp_approach = make_translation(0.06, 0.1, [0.0, 0.0, -1.0])
    template.post_grasp_retreat = make_translation(0.06, 0.1, [0.0, 0.0, 1.0])
    template.allowed_touch_objects = [target_id, support_surface]
    yaw = atan2(target_pose.pose.position.y, target_pose.pose.position.x)
    grasps = []
    for offset in (0.0, 0.03, 0.05, -0.03, -0.05):
        grasp = deepcopy(template)
        grasp.id = str(len(grasps))
        quaternion = quaternion_from_euler(0.0, pi / 2, yaw + offset)
        (
            grasp.grasp_pose.pose.orientation.x,
            grasp.grasp_pose.pose.orientation.y,
            grasp.grasp_pose.pose.orientation.z,
            grasp.grasp_pose.pose.orientation.w,
        ) = quaternion
        grasps.append(grasp)
    return grasps


def make_places(place_pose):
    template = PlaceLocation()
    template.place_pose = deepcopy(place_pose)
    template.pre_place_approach = make_translation(0.05, 0.08, [0.0, 0.0, -1.0])
    template.post_place_retreat = make_translation(0.06, 0.08, [0.0, 0.0, 1.0])
    template.post_place_posture = make_gripper_posture(GRIPPER_OPEN)
    places = []
    for x_offset in (0.0, 0.005, -0.005):
        for y_offset in (0.0, 0.005, -0.005):
            place = deepcopy(template)
            place.place_pose.pose.position.x += x_offset
            place.place_pose.pose.position.y += y_offset
            places.append(place)
    return places


class ArucoPickServer(Node):
    def __init__(self):
        super().__init__("aruco_pick_server")
        self.latest_markers = {}
        self.moveit = MoveItPy(node_name="aruco_pick_server_moveit")
        self.monitor = self.moveit.get_planning_scene_monitor()
        self.arm = self.moveit.get_planning_component(ARM_GROUP)
        self.pickup_client = ActionClient(self, Pickup, "/pickup")
        self.place_client = ActionClient(self, Place, "/place")
        self.create_subscription(
            MarkerPoseArray, "/aruco_markers", self.marker_callback, 10
        )
        self.create_service(SetBool, "/xarm_vision_pickup", self.pick_callback)

    def marker_callback(self, message):
        self.latest_markers = {marker.id: deepcopy(marker.pose) for marker in message.markers}

    async def pick_callback(self, request, response):
        if not request.data:
            self._clear_scene()
            response.success = True
            response.message = "Planning scene cleared"
            return response
        if not self.latest_markers:
            response.success = False
            response.message = "No ArUco markers detected"
            return response

        table_id = "table"
        self._add_box(table_id, self._pose(0.0, 0.0, -0.005), [1.0, 1.2, 0.01])
        set_named_goal(self.arm, "Home")
        if not plan_and_execute(self.moveit, self.arm, self):
            response.success = False
            response.message = "Failed to plan Home"
            return response

        place_height = 0.035
        processed_ids = []
        for marker_id, marker_pose in sorted(self.latest_markers.items()):
            target_id = f"aruco_{marker_id}"
            self._add_box(target_id, marker_pose, [0.07, 0.07, 0.07])
            pickup_goal = Pickup.Goal()
            pickup_goal.target_name = target_id
            pickup_goal.group_name = ARM_GROUP
            pickup_goal.end_effector = GRIPPER_GROUP
            pickup_goal.possible_grasps = make_grasps(marker_pose, target_id, table_id)
            pickup_goal.support_surface_name = table_id
            pickup_goal.attached_object_touch_links = ["gripper_1_link", "gripper_2_link"]
            pickup_goal.allowed_planning_time = 10.0
            pickup_goal.planning_options.replan = True
            if await self._execute_action(self.pickup_client, pickup_goal) != MoveItErrorCodes.SUCCESS:
                response.success = False
                response.message = f"Pickup failed for marker {marker_id}"
                return response

            place_pose = self._pose(0.25, 0.25, place_height)
            place_goal = Place.Goal()
            place_goal.group_name = ARM_GROUP
            place_goal.attached_object_name = target_id
            place_goal.place_locations = make_places(place_pose)
            place_goal.support_surface_name = table_id
            place_goal.allowed_planning_time = 10.0
            place_goal.planning_options.replan = True
            if await self._execute_action(self.place_client, place_goal) != MoveItErrorCodes.SUCCESS:
                response.success = False
                response.message = f"Place failed for marker {marker_id}"
                return response
            processed_ids.append(target_id)
            place_height += 0.07

        set_named_goal(self.arm, "Home")
        plan_and_execute(self.moveit, self.arm, self)
        self._remove_object(table_id)
        for target_id in processed_ids:
            self._remove_object(target_id)
        response.success = True
        response.message = f"Picked and placed {len(processed_ids)} marker(s)"
        return response

    async def _execute_action(self, client, goal):
        if not client.wait_for_server(timeout_sec=10.0):
            return MoveItErrorCodes.FAILURE
        goal_handle = await client.send_goal_async(goal)
        if not goal_handle.accepted:
            return MoveItErrorCodes.FAILURE
        wrapped_result = await goal_handle.get_result_async()
        return wrapped_result.result.error_code.val

    def _pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = REFERENCE_FRAME
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
        time.sleep(0.1)

    def _remove_object(self, object_id):
        collision = CollisionObject()
        collision.id = object_id
        collision.operation = CollisionObject.REMOVE
        self.monitor.process_collision_object(collision)

    def _clear_scene(self):
        self.monitor.remove_all_collision_objects()

    def destroy_node(self):
        self.pickup_client.destroy()
        self.place_client.destroy()
        self.moveit.shutdown()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPickServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
