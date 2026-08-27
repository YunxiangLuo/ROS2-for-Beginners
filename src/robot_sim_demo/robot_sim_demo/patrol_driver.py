"""Publish a repeatable differential-drive patrol pattern."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class PatrolDriver(Node):
    def __init__(self) -> None:
        super().__init__("patrol_driver")
        self.declare_parameter("linear_speed", 0.18)
        self.declare_parameter("angular_speed", 0.55)
        self.declare_parameter("loop", True)
        self.declare_parameter("duration", 0.0)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.duration = max(float(self.get_parameter("duration").value), 0.0)
        self.started_at = time.monotonic()
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.05, self.publish_command)
        self.finished = False

        self.get_logger().info(
            "Patrol driver ready: "
            f"speed={self.linear_speed:.2f} m/s, "
            f"turn={self.angular_speed:.2f} rad/s, loop={self.loop}"
        )

    def publish_command(self) -> None:
        elapsed = time.monotonic() - self.started_at
        if self.duration and elapsed >= self.duration and not self.loop:
            self.stop()
            self.finished = True
            self.timer.cancel()
            return

        # Four straight legs with 90-degree turns, repeated for the recording.
        phase = elapsed % 31.4
        command = Twist()
        if phase < 5.0:
            command.linear.x = self.linear_speed
        elif phase < 7.85:
            command.angular.z = self.angular_speed
        elif phase < 12.85:
            command.linear.x = self.linear_speed
        elif phase < 15.70:
            command.angular.z = self.angular_speed
        elif phase < 20.70:
            command.linear.x = self.linear_speed
        elif phase < 23.55:
            command.angular.z = self.angular_speed
        elif phase < 28.55:
            command.linear.x = self.linear_speed
        else:
            command.angular.z = self.angular_speed
        self.publisher.publish(command)

    def stop(self) -> None:
        if rclpy.ok():
            self.publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
