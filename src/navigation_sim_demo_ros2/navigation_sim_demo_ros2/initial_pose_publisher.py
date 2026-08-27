import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("initial_pose_publisher")
        self.declare_parameter("topic", "/initialpose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("publish_count", 10)
        self.declare_parameter("publish_period_sec", 0.5)

        self.topic = str(self.get_parameter("topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.yaw = float(self.get_parameter("yaw").value)
        self.publish_count = int(self.get_parameter("publish_count").value)
        period = float(self.get_parameter("publish_period_sec").value)

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, self.topic, 10)
        self.remaining_publishes = self.publish_count
        self.timer = self.create_timer(period, self.publish_initial_pose)

    def publish_initial_pose(self) -> None:
        if self.remaining_publishes <= 0:
            self.timer.cancel()
            return

        message = PoseWithCovarianceStamped()
        message.header.frame_id = self.frame_id
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        quat = yaw_to_quaternion(self.yaw)
        message.pose.pose.orientation.z = quat[2]
        message.pose.pose.orientation.w = quat[3]
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.06853891945200942
        self.publisher.publish(message)
        self.remaining_publishes -= 1


def main() -> None:
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
