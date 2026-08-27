"""Verify the running arm-only controllers and MoveIt planning pipeline."""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


EXPECTED_CONTROLLERS = {
    "joint_state_broadcaster",
    "xarm_controller",
    "gripper_controller",
}
ARM_JOINTS = tuple(f"arm_{index}_joint" for index in range(1, 7))
TARGET_POSITIONS = (0.20, -0.20, 0.20, -0.20, 0.10, 0.0)


class RuntimeSmoke(Node):
    def __init__(self) -> None:
        super().__init__("arm_only_runtime_smoke")
        self._joint_state: JointState | None = None
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self._controllers = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._planner = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._executor = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

    def _on_joint_state(self, message: JointState) -> None:
        self._joint_state = message

    def _wait_for_joint_state(self, timeout_sec: float) -> JointState:
        deadline = time.monotonic() + timeout_sec
        while self._joint_state is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._joint_state is None:
            raise RuntimeError("No /joint_states message received")
        return self._joint_state

    def _call(self, client, request, timeout_sec: float):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Service {client.srv_name} is unavailable")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"Service {client.srv_name} timed out")
        return future.result()

    def verify_controllers(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        missing = set(EXPECTED_CONTROLLERS)
        active: set[str] = set()
        while missing and time.monotonic() < deadline:
            remaining = max(0.1, deadline - time.monotonic())
            response = self._call(
                self._controllers,
                ListControllers.Request(),
                min(2.0, remaining),
            )
            active = {
                controller.name
                for controller in response.controller
                if controller.state == "active"
            }
            missing = EXPECTED_CONTROLLERS - active
            if missing:
                time.sleep(0.25)
        if missing:
            raise RuntimeError(f"Inactive controllers: {', '.join(sorted(missing))}")
        print("Active controllers: " + ", ".join(sorted(active)))

    def verify_joint_states(self, timeout_sec: float) -> None:
        state = self._wait_for_joint_state(timeout_sec)
        missing = set(ARM_JOINTS) - set(state.name)
        if missing:
            raise RuntimeError(f"Missing joint states: {', '.join(sorted(missing))}")
        print("Joint states: " + ", ".join(state.name))

    def plan(self, timeout_sec: float):
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = "xarm"
        motion.num_planning_attempts = 1
        motion.allowed_planning_time = 5.0
        motion.max_velocity_scaling_factor = 0.5
        motion.max_acceleration_scaling_factor = 0.5
        motion.start_state.is_diff = True
        motion.goal_constraints = [
            Constraints(
                joint_constraints=[
                    JointConstraint(
                        joint_name=name,
                        position=position,
                        tolerance_above=0.01,
                        tolerance_below=0.01,
                        weight=1.0,
                    )
                    for name, position in zip(ARM_JOINTS, TARGET_POSITIONS)
                ]
            )
        ]
        response = self._call(self._planner, request, timeout_sec)
        result = response.motion_plan_response
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"MoveIt planning failed with code {result.error_code.val}")
        point_count = len(result.trajectory.joint_trajectory.points)
        if point_count == 0:
            raise RuntimeError("MoveIt returned an empty trajectory")
        print(f"MoveIt plan: SUCCESS ({point_count} trajectory points)")
        return result.trajectory

    def execute(self, trajectory, timeout_sec: float) -> None:
        if not self._executor.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError("Action /execute_trajectory is unavailable")
        goal = ExecuteTrajectory.Goal(trajectory=trajectory)
        goal_future = self._executor.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=timeout_sec)
        if not goal_future.done() or goal_future.result() is None:
            raise RuntimeError("MoveIt trajectory goal timed out")
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("MoveIt trajectory goal was rejected")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done() or result_future.result() is None:
            raise RuntimeError("MoveIt trajectory execution timed out")
        error_code = result_future.result().result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"MoveIt execution failed with code {error_code}")
        print("MoveIt execution: SUCCESS")


def main(args: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Execute the planned test trajectory through MoveIt",
    )
    parser.add_argument("--timeout", type=float, default=15.0)
    options = parser.parse_args(args)

    rclpy.init(args=[])
    node = RuntimeSmoke()
    try:
        node.verify_controllers(options.timeout)
        node.verify_joint_states(options.timeout)
        trajectory = node.plan(options.timeout)
        if options.execute:
            node.execute(trajectory, options.timeout)
    except RuntimeError as error:
        print(f"Runtime smoke test failed: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
