import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_demo_interfaces.action import DoDishes


class DoDishesClient(Node):
    def __init__(self) -> None:
        super().__init__("dishes_client")
        self._client = ActionClient(self, DoDishes, "dishes")
        self._done = False
        self._exit_code = 0

    def run(self) -> int:
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available")
            return 1

        goal_msg = DoDishes.Goal()
        goal_msg.dishwasher_id = 2

        future = self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

        while rclpy.ok() and not self._done:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self._exit_code

    def feedback_callback(self, feedback_msg) -> None:
        self.get_logger().info(
            f"Feedback {feedback_msg.feedback.percent_complete:.1f}%"
        )

    def goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by server")
            self._done = True
            self._exit_code = 1
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future) -> None:
        result = future.result().result
        self.get_logger().info(
            f"Yay! The dishes are now clean. Total cleaned: {result.total_dishes_cleaned}"
        )
        self._done = True


def main() -> None:
    rclpy.init()
    node = DoDishesClient()
    try:
        raise SystemExit(node.run())
    finally:
        node.destroy_node()
        rclpy.shutdown()
