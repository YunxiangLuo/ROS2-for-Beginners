import time

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
import rclpy
from rclpy.node import Node


LOCALIZATION_NODES = ["map_server", "amcl"]
NAVIGATION_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
]


class Nav2LifecycleRunner(Node):
    def __init__(self) -> None:
        super().__init__("nav2_lifecycle_runner")
        self.declare_parameter("configure_timeout_sec", 10.0)
        self.declare_parameter("activate_timeout_sec", 10.0)
        self.declare_parameter("service_wait_timeout_sec", 20.0)
        self.declare_parameter("retry_count", 5)

        self.configure_timeout_sec = float(self.get_parameter("configure_timeout_sec").value)
        self.activate_timeout_sec = float(self.get_parameter("activate_timeout_sec").value)
        self.service_wait_timeout_sec = float(self.get_parameter("service_wait_timeout_sec").value)
        self.retry_count = int(self.get_parameter("retry_count").value)

        self.lifecycle_clients: dict[str, tuple] = {}
        for node_name in LOCALIZATION_NODES + NAVIGATION_NODES:
            change_client = self.create_client(ChangeState, f"/{node_name}/change_state")
            state_client = self.create_client(GetState, f"/{node_name}/get_state")
            self.lifecycle_clients[node_name] = (change_client, state_client)

    def wait_for_services(self) -> None:
        deadline = time.monotonic() + self.service_wait_timeout_sec
        for node_name, (change_client, state_client) in self.lifecycle_clients.items():
            while time.monotonic() < deadline:
                if change_client.wait_for_service(timeout_sec=0.5) and state_client.wait_for_service(timeout_sec=0.5):
                    break
            else:
                raise RuntimeError(f"Timed out waiting for lifecycle services of {node_name}")

    def get_state(self, node_name: str) -> int:
        _, state_client = self.lifecycle_clients[node_name]
        for attempt in range(1, self.retry_count + 1):
            request = GetState.Request()
            future = state_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            response = future.result()
            if response is not None:
                return int(response.current_state.id)

            self.get_logger().warn(
                f"Failed to get lifecycle state for {node_name} "
                f"on attempt {attempt}/{self.retry_count}"
            )
            time.sleep(0.2)

        raise RuntimeError(f"Failed to get lifecycle state for {node_name}")

    def change_state(self, node_name: str, transition_id: int) -> bool:
        change_client, _ = self.lifecycle_clients[node_name]
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = change_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        return bool(response and response.success)

    def wait_for_state(self, node_name: str, target_state: int, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.get_state(node_name) == target_state:
                return True
            time.sleep(0.1)
        return False

    def transition_node(self, node_name: str, transition_id: int, target_state: int, timeout_sec: float) -> None:
        if self.get_state(node_name) == target_state:
            return

        for attempt in range(1, self.retry_count + 1):
            accepted = self.change_state(node_name, transition_id)
            if not accepted and self.get_state(node_name) == target_state:
                return
            if not accepted:
                self.get_logger().warn(
                    f"{node_name} rejected transition {transition_id} on attempt {attempt}/{self.retry_count}"
                )
                time.sleep(0.2)
                continue

            if self.wait_for_state(node_name, target_state, timeout_sec):
                return

            if self.get_state(node_name) == target_state:
                return

            self.get_logger().warn(
                f"{node_name} did not reach state {target_state} after transition {transition_id} "
                f"on attempt {attempt}/{self.retry_count}"
            )
            time.sleep(0.2)

        raise RuntimeError(f"Failed to transition {node_name} to state {target_state}")

    def bringup_group(self, node_names: list[str]) -> None:
        for node_name in node_names:
            self.transition_node(
                node_name=node_name,
                transition_id=Transition.TRANSITION_CONFIGURE,
                target_state=State.PRIMARY_STATE_INACTIVE,
                timeout_sec=self.configure_timeout_sec,
            )
        for node_name in node_names:
            self.transition_node(
                node_name=node_name,
                transition_id=Transition.TRANSITION_ACTIVATE,
                target_state=State.PRIMARY_STATE_ACTIVE,
                timeout_sec=self.activate_timeout_sec,
            )

    def bringup(self) -> None:
        self.wait_for_services()
        self.get_logger().info("Bringing up localization lifecycle nodes")
        self.bringup_group(LOCALIZATION_NODES)
        self.get_logger().info("Bringing up navigation lifecycle nodes")
        self.bringup_group(NAVIGATION_NODES)
        self.get_logger().info("Nav2 stack is active")
        print("nav2-stack-active")


def main() -> None:
    rclpy.init()
    node = Nav2LifecycleRunner()
    try:
        node.bringup()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
