import rclpy
from rclpy.node import Node

from topic_demo_interfaces.msg import Gps


class GpsTalker(Node):
    def __init__(self) -> None:
        super().__init__("pytalker")
        self.publisher = self.create_publisher(Gps, "gps_info", 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.x = 1.0
        self.y = 2.0
        self.state = "working"

    def publish_message(self) -> None:
        msg = Gps()
        msg.state = self.state
        msg.x = float(self.x)
        msg.y = float(self.y)
        self.get_logger().info(f"Talker: GPS: x={msg.x:.6f}, y={msg.y:.6f}")
        self.publisher.publish(msg)
        self.x *= 1.03
        self.y *= 1.01


def main() -> None:
    rclpy.init()
    node = GpsTalker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
