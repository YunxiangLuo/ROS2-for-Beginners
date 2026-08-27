"""Broadcast and query deterministic bottle transforms."""

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


MARKERS = (
    ("ar_marker_hcl", -0.3, 0.2, 0.1),
    ("ar_marker_naoh", 0.3, -0.2, 0.1),
    ("ar_marker_h2o", 0.5, 0.3, 0.1),
    ("ar_marker_phenolphthalein", -0.5, 0.0, 0.1),
)


class BottleLocalizer(Node):
    def __init__(self):
        super().__init__("bottle_localizer")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0, self.broadcast_markers)
        self.create_timer(2.0, self.localize_bottles)

    def broadcast_markers(self):
        stamp = self.get_clock().now().to_msg()
        for frame_id, x, y, z in MARKERS:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = "base_link"
            transform.child_frame_id = frame_id
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            transform.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(transform)

    def localize_bottles(self):
        for frame_id, *_ in MARKERS:
            try:
                transform = self.tf_buffer.lookup_transform("base_link", frame_id, Time())
            except Exception as error:
                self.get_logger().debug(f"{frame_id}: {error}")
                continue
            translation = transform.transform.translation
            self.get_logger().info(
                f"{frame_id}: x={translation.x:.3f}, y={translation.y:.3f}, "
                f"z={translation.z:.3f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = BottleLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
