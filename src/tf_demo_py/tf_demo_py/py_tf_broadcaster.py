from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .math_utils import quaternion_from_euler


class TfBroadcasterNode(Node):
    def __init__(self) -> None:
        super().__init__("py_tf_broadcaster")
        self.broadcaster = TransformBroadcaster(self)
        self.yaw = 1.57
        self.timer = self.create_timer(1.0, self.broadcast)

    def broadcast(self) -> None:
        self.yaw += 0.1
        quat = quaternion_from_euler(0.0, 0.0, self.yaw)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "link1"
        t.transform.translation.x = 1.0
        t.transform.translation.y = 2.0
        t.transform.translation.z = 3.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.broadcaster.sendTransform(t)
        self.get_logger().info(f"Broadcasted transform yaw={self.yaw:.2f}")


def main() -> None:
    rclpy.init()
    node = TfBroadcasterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
