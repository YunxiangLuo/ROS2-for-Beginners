import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException


class TfListenerNode(Node):
    def __init__(self) -> None:
        super().__init__("py_tf_listener")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(1.0, self.lookup)

    def lookup(self) -> None:
        try:
            transform = self.buffer.lookup_transform("base_link", "link1", rclpy.time.Time())
            tr = transform.transform.translation
            rot = transform.transform.rotation
            self.get_logger().info(
                "translation x=%.2f y=%.2f z=%.2f | quaternion w=%.4f x=%.4f y=%.4f z=%.4f"
                % (tr.x, tr.y, tr.z, rot.w, rot.x, rot.y, rot.z)
            )
        except TransformException as ex:
            self.get_logger().warn(f"Transform lookup failed: {ex}")


def main() -> None:
    rclpy.init()
    node = TfListenerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
