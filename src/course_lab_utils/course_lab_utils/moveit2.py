"""Small adapters around the supported ROS 2 Jazzy MoveItPy API."""

from copy import deepcopy
from math import cos, sin

import numpy as np
import rclpy
from moveit.core.robot_state import RobotState, robotStateToRobotStateMsg
from moveit.core.robot_trajectory import RobotTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
    """Return an (x, y, z, w) quaternion without an extra TF dependency."""
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def set_named_goal(component, configuration_name: str) -> None:
    component.set_start_state_to_current_state()
    component.set_goal_state(configuration_name=configuration_name)


def set_pose_goal(component, pose_stamped, pose_link: str) -> None:
    component.set_start_state_to_current_state()
    component.set_goal_state(pose_stamped_msg=pose_stamped, pose_link=pose_link)


def set_joint_goal(moveit, component, group_name: str, positions) -> RobotState:
    values = np.asarray(positions, dtype=float)
    state = RobotState(moveit.get_robot_model())
    state.set_to_default_values()
    state.set_joint_group_active_positions(group_name, values)
    state.update()
    component.set_start_state_to_current_state()
    component.set_goal_state(robot_state=state)
    return state


def plan_and_execute(moveit, component) -> bool:
    result = component.plan()
    if not result:
        return False
    moveit.execute(result.trajectory, controllers=[])
    return True


def get_current_pose(moveit, link_name: str):
    monitor = moveit.get_planning_scene_monitor()
    with monitor.read_only() as scene:
        return deepcopy(scene.current_state.get_pose(link_name))


def build_cartesian_request(
    start_state,
    frame_id: str,
    group_name: str,
    link_name: str,
    waypoints,
    max_step: float = 0.01,
    jump_threshold: float = 0.0,
    avoid_collisions: bool = True,
):
    request = GetCartesianPath.Request()
    request.header.frame_id = frame_id
    request.start_state = start_state
    request.group_name = group_name
    request.link_name = link_name
    request.waypoints = list(waypoints)
    request.max_step = max_step
    request.jump_threshold = jump_threshold
    request.avoid_collisions = avoid_collisions
    return request


def compute_cartesian_path(
    node,
    moveit,
    group_name: str,
    link_name: str,
    frame_id: str,
    waypoints,
    timeout_sec: float = 10.0,
):
    """Call MoveIt's supported Cartesian path service and return fraction/trajectory."""
    client = node.create_client(GetCartesianPath, "/compute_cartesian_path")
    try:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("MoveIt Cartesian path service is unavailable")

        monitor = moveit.get_planning_scene_monitor()
        with monitor.read_only() as scene:
            start_state = robotStateToRobotStateMsg(scene.current_state)
        request = build_cartesian_request(
            start_state,
            frame_id,
            group_name,
            link_name,
            waypoints,
        )
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        response = future.result()
        if response is None:
            raise RuntimeError("MoveIt Cartesian path request timed out")
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return response.fraction, None

        trajectory = RobotTrajectory(moveit.get_robot_model())
        with monitor.read_only() as scene:
            trajectory.set_robot_trajectory_msg(scene.current_state, response.solution)
        return response.fraction, trajectory
    finally:
        node.destroy_client(client)


def scene_object_ids(planning_scene_monitor) -> set[str]:
    with planning_scene_monitor.read_only() as scene:
        world = scene.planning_scene_message.world.collision_objects
        return {collision_object.id for collision_object in world}


def attached_object_ids(planning_scene_monitor) -> set[str]:
    with planning_scene_monitor.read_only() as scene:
        attached = scene.planning_scene_message.robot_state.attached_collision_objects
        return {attached_object.object.id for attached_object in attached}
