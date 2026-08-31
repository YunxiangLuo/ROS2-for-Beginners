"""Execute the xArm MoveIt demonstration sequence through Gazebo controllers."""
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

import rclpy
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = tuple(f"arm_{index}_joint" for index in range(1, 7))
GRIPPER_JOINTS = ("gripper_1_joint", "gripper_2_joint")
EXPECTED_CONTROLLERS = {
    "joint_state_broadcaster",
    "xarm_controller",
    "gripper_controller",
}


@dataclass(frozen=True)
class Step:
    name: str
    group: str
    targets: tuple[tuple[str, float], ...]


SEQUENCE = (
    Step("Home", "xarm", tuple((joint, 0.0) for joint in ARM_JOINTS)),
    Step(
        "Down",
        "xarm",
        tuple(
            zip(
                ARM_JOINTS,
                (0.0, -1.0, 0.0, -1.5, 0.0, -0.55),
            )
        ),
    ),
    Step(
        "Open_gripper",
        "gripper",
        tuple((joint, 0.68) for joint in GRIPPER_JOINTS),
    ),
    Step(
        "Close_gripper",
        "gripper",
        tuple((joint, 0.0) for joint in GRIPPER_JOINTS),
    ),
    Step("Home", "xarm", tuple((joint, 0.0) for joint in ARM_JOINTS)),
)


class MoveItSequence(Node):
    def __init__(self, step_timeout: float) -> None:
        super().__init__("xarm_moveit_sequence")
        self.step_timeout = step_timeout
        self.joint_positions: dict[str, float] = {}
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.controllers = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.planner = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

    def _on_joint_state(self, message: JointState) -> None:
        self.joint_positions.update(zip(message.name, message.position))

    def _wait_for_controllers(self) -> None:
        deadline = time.monotonic() + self.step_timeout
        while time.monotonic() < deadline:
            if not self.controllers.wait_for_service(timeout_sec=1.0):
                continue
            future = self.controllers.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            response = future.result()
            if response is not None:
                active = {
                    controller.name
                    for controller in response.controller
                    if controller.state == "active"
                }
                missing = EXPECTED_CONTROLLERS - active
                if not missing:
                    print("Active controllers: " + ", ".join(sorted(active)))
                    return
            time.sleep(0.25)
        raise RuntimeError("Required xArm controllers did not become active")

    def _wait_for_joint_state(self) -> None:
        deadline = time.monotonic() + self.step_timeout
        required = set(ARM_JOINTS) | set(GRIPPER_JOINTS)
        while not required.issubset(self.joint_positions) and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        missing = required - self.joint_positions.keys()
        if missing:
            raise RuntimeError(f"Missing joint states: {', '.join(sorted(missing))}")

    def _plan(self, step: Step):
        if not self.planner.wait_for_service(timeout_sec=self.step_timeout):
            raise RuntimeError("MoveIt planning service is unavailable")

        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = step.group
        motion.num_planning_attempts = 3
        motion.allowed_planning_time = 5.0
        motion.max_velocity_scaling_factor = 0.35
        motion.max_acceleration_scaling_factor = 0.35
        # Send a complete, clamped start state so tiny simulator rounding at a
        # hard joint limit cannot make MoveIt reject the next request.
        motion.start_state.is_diff = False
        current_names = list(ARM_JOINTS) + list(GRIPPER_JOINTS)
        motion.start_state.joint_state.name = current_names
        motion.start_state.joint_state.position = [
            max(0.0, min(0.68, self.joint_positions[name]))
            if name in GRIPPER_JOINTS
            else self.joint_positions[name]
            for name in current_names
        ]
        motion.goal_constraints = [
            Constraints(
                joint_constraints=[
                    JointConstraint(
                        joint_name=joint,
                        position=position,
                        tolerance_above=0.01,
                        tolerance_below=0.01,
                        weight=1.0,
                    )
                    for joint, position in step.targets
                ]
            )
        ]
        future = self.planner.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.step_timeout)
        response = future.result()
        if response is None:
            raise RuntimeError(f"Planning timed out for {step.name}")
        result = response.motion_plan_response
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"Planning failed for {step.name}: {result.error_code.val}"
            )
        if not result.trajectory.joint_trajectory.points:
            raise RuntimeError(f"MoveIt returned an empty trajectory for {step.name}")
        if step.group == "gripper":
            self._ensure_gripper_start(result.trajectory)
        return result.trajectory

    def _ensure_gripper_start(self, trajectory) -> None:
        joint_trajectory = trajectory.joint_trajectory
        current = [self.joint_positions[name] for name in joint_trajectory.joint_names]
        first = joint_trajectory.points[0]
        first_time = first.time_from_start.sec + first.time_from_start.nanosec / 1e9
        first_is_current = (
            len(first.positions) == len(current)
            and all(abs(position - actual) <= 0.01 for position, actual in zip(first.positions, current))
        )
        if first_time <= 1e-6 and first_is_current:
            return
        if first_time <= 1e-6:
            first.time_from_start.nanosec = 100_000_000
        start = JointTrajectoryPoint()
        start.positions = current
        joint_trajectory.points.insert(0, start)

    def _execute(self, step: Step, trajectory) -> None:
        if not self._execute_client.wait_for_server(timeout_sec=self.step_timeout):
            raise RuntimeError("MoveIt execute trajectory action is unavailable")
        goal_future = self._execute_client.send_goal_async(
            ExecuteTrajectory.Goal(trajectory=trajectory)
        )
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=self.step_timeout)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"Execution goal rejected for {step.name}")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self.step_timeout
        )
        action_result = result_future.result()
        if action_result is None:
            raise RuntimeError(f"Execution timed out for {step.name}")
        error_code = action_result.result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"Execution failed for {step.name}: {error_code}")

    def _wait_for_targets(self, step: Step) -> None:
        deadline = time.monotonic() + self.step_timeout
        while time.monotonic() < deadline:
            if all(
                joint in self.joint_positions
                and abs(self.joint_positions[joint] - target) <= 0.03
                for joint, target in step.targets
            ):
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(f"Joint feedback did not reach {step.name}")

    def run(self) -> None:
        self._wait_for_controllers()
        self._wait_for_joint_state()
        for step in SEQUENCE:
            print(f"Planning {step.name} ({step.group})...")
            trajectory = self._plan(step)
            self._execute(step, trajectory)
            self._wait_for_targets(step)
            print(f"Completed {step.name}")
            time.sleep(1.0)


def main(args: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=30.0)
    options = parser.parse_args(args)

    rclpy.init(args=[])
    node = MoveItSequence(options.timeout)
    try:
        node.run()
    except RuntimeError as error:
        print(f"xArm MoveIt sequence failed: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("xArm MoveIt sequence completed: Home -> Down -> Open -> Close -> Home")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
