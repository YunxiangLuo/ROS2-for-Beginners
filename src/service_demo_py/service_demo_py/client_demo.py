import rclpy
from rclpy.node import Node

from service_demo_interfaces.srv import Greeting


class GreetingClient(Node):
    def __init__(self) -> None:
        super().__init__("greetings_client")
        self.client = self.create_client(Greeting, "greetings")

    def run(self) -> int:
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for the service")
                return 1
            self.get_logger().info("Waiting for service...")

        request = Greeting.Request()
        request.name = "HAN"
        request.age = 20
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("Service call failed")
            return 1

        self.get_logger().info(f"Message From server: {future.result().feedback}")
        return 0


def main() -> None:
    rclpy.init()
    node = GreetingClient()
    try:
        raise SystemExit(node.run())
    finally:
        node.destroy_node()
        rclpy.shutdown()
