from geometry_msgs.msg import TransformStamped
import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def compute_target_position(
    motion_mode: str,
    elapsed_sec: float,
    x: float,
    y: float,
    z: float,
    center_x: float,
    center_y: float,
    radius: float,
    angular_speed: float,
) -> tuple[float, float, float]:
    if motion_mode == "circle":
        angle = angular_speed * elapsed_sec
        return (
            center_x + radius * math.cos(angle),
            center_y + radius * math.sin(angle),
            z,
        )
    return (x, y, z)


class FakeTargetBroadcasterNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_target_broadcaster")
        self.declare_parameter("parent_frame", "mybot_link")
        self.declare_parameter("child_frame", "base_footprint")
        self.declare_parameter("motion_mode", "static")
        self.declare_parameter("x", 3.0)
        self.declare_parameter("y", 1.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("center_x", 3.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("radius", 1.0)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("period_sec", 0.1)

        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.child_frame = str(self.get_parameter("child_frame").value)
        self.motion_mode = str(self.get_parameter("motion_mode").value)
        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.z = float(self.get_parameter("z").value)
        self.center_x = float(self.get_parameter("center_x").value)
        self.center_y = float(self.get_parameter("center_y").value)
        self.radius = float(self.get_parameter("radius").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.period_sec = max(float(self.get_parameter("period_sec").value), 0.01)

        self.broadcaster = TransformBroadcaster(self)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(self.period_sec, self.publish_transform)

    def publish_transform(self) -> None:
        elapsed_sec = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        x, y, z = compute_target_position(
            motion_mode=self.motion_mode,
            elapsed_sec=elapsed_sec,
            x=self.x,
            y=self.y,
            z=self.z,
            center_x=self.center_x,
            center_y=self.center_y,
            radius=self.radius,
            angular_speed=self.angular_speed,
        )
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = FakeTargetBroadcasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
