from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from .controller import compute_follow_command


class TfFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("tf_follower")
        self.declare_parameter("follower_frame", "mybot_link")
        self.declare_parameter("target_frame", "base_footprint")
        self.declare_parameter("cmd_vel_topic", "/mybot_cmd_vel")
        self.declare_parameter("stop_distance", 1.0)
        self.declare_parameter("linear_gain", 0.1)
        self.declare_parameter("angular_gain", -0.4)
        self.declare_parameter("max_linear_speed", 1.0)
        self.declare_parameter("max_angular_speed", 1.5)
        self.declare_parameter("lookup_rate_hz", 10.0)

        self.follower_frame = str(self.get_parameter("follower_frame").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.lookup_rate_hz = max(float(self.get_parameter("lookup_rate_hz").value), 1.0)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(1.0 / self.lookup_rate_hz, self.follow_target)
        self._last_lookup_failed = False

        self.get_logger().info(
            "Following '%s' from '%s' and publishing Twist on '%s'"
            % (self.target_frame, self.follower_frame, cmd_vel_topic)
        )

    def follow_target(self) -> None:
        try:
            transform = self.buffer.lookup_transform(
                self.follower_frame, self.target_frame, rclpy.time.Time()
            )
        except TransformException as ex:
            if not self._last_lookup_failed:
                self.get_logger().warn(f"Transform lookup failed: {ex}")
            self._last_lookup_failed = True
            return

        self._last_lookup_failed = False
        translation = transform.transform.translation
        linear_x, angular_z = compute_follow_command(
            x=translation.x,
            y=translation.y,
            stop_distance=self.stop_distance,
            linear_gain=self.linear_gain,
            angular_gain=self.angular_gain,
            max_linear_speed=self.max_linear_speed,
            max_angular_speed=self.max_angular_speed,
        )

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.publisher.publish(cmd)

    def publish_stop(self) -> None:
        self.publisher.publish(Twist())


def main() -> None:
    rclpy.init()
    node = TfFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
