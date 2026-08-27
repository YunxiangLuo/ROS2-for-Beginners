import rclpy
from rclpy.node import Node

from service_demo_interfaces.srv import Greeting


class GreetingServer(Node):
    def __init__(self) -> None:
        super().__init__("greetings_server")
        self.service = self.create_service(Greeting, "greetings", self.handle_request)
        self.get_logger().info("Ready to handle the request")

    def handle_request(
        self, request: Greeting.Request, response: Greeting.Response
    ) -> Greeting.Response:
        self.get_logger().info(
            f"Request from {request.name} with age {request.age}"
        )
        response.feedback = f"Hi {request.name}. I'm server!"
        return response


def main() -> None:
    rclpy.init()
    node = GreetingServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
