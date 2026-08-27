import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from action_demo_interfaces.action import DoDishes


class DoDishesServer(Node):
    def __init__(self) -> None:
        super().__init__("dishes_server")
        self._action_server = ActionServer(
            self,
            DoDishes,
            "dishes",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def goal_callback(self, goal_request: DoDishes.Goal) -> GoalResponse:
        self.get_logger().info(
            f"Received goal for dishwasher_id={goal_request.dishwasher_id}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Received request to cancel goal")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        feedback = DoDishes.Feedback()
        result = DoDishes.Result()

        for step in range(1, 6):
            if goal_handle.is_cancel_requested:
                result.total_dishes_cleaned = (step - 1) * goal_handle.request.dishwasher_id
                goal_handle.canceled()
                return result

            feedback.percent_complete = float(step * 20)
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"Feedback {feedback.percent_complete:.1f}%")
            time.sleep(0.5)

        result.total_dishes_cleaned = goal_handle.request.dishwasher_id * 5
        goal_handle.succeed()
        self.get_logger().info(
            f"Goal succeeded, cleaned {result.total_dishes_cleaned} dishes"
        )
        return result


def main() -> None:
    rclpy.init()
    node = DoDishesServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
