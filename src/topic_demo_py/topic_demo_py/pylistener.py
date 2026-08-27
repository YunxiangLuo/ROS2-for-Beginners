import math

import rclpy
from rclpy.node import Node

from topic_demo_interfaces.msg import Gps


class GpsListener(Node):
    def __init__(self) -> None:
        super().__init__("pylistener")
        self.subscription = self.create_subscription(
            Gps, "gps_info", self.handle_message, 10
        )

    def handle_message(self, msg: Gps) -> None:
        distance = math.sqrt((msg.x ** 2) + (msg.y ** 2))
        self.get_logger().info(
            f"Listener: GPS: distance={distance:.6f}, state={msg.state}"
        )


def main() -> None:
    rclpy.init()
    node = GpsListener()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
