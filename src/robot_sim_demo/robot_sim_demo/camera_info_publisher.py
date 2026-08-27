"""Publish the calibrated CameraInfo matching the Gazebo camera sensor."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import CameraInfo


class CameraInfoPublisher(Node):
    def __init__(self) -> None:
        super().__init__("camera_info_publisher")
        self.declare_parameter("topic", "/camera/camera_info")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("width", 320)
        self.declare_parameter("height", 180)
        self.declare_parameter("horizontal_fov", 1.0472)
        self.declare_parameter("publish_rate", 3.0)

        self.topic = str(self.get_parameter("topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        horizontal_fov = float(self.get_parameter("horizontal_fov").value)
        publish_rate = max(float(self.get_parameter("publish_rate").value), 0.1)

        focal_length = (self.width * 0.5) / math.tan(horizontal_fov * 0.5)
        self.camera_info = CameraInfo()
        self.camera_info.width = self.width
        self.camera_info.height = self.height
        self.camera_info.distortion_model = "plumb_bob"
        self.camera_info.d = [0.0] * 5
        self.camera_info.k = [
            focal_length,
            0.0,
            (self.width - 1) * 0.5,
            0.0,
            focal_length,
            (self.height - 1) * 0.5,
            0.0,
            0.0,
            1.0,
        ]
        self.camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info.p = [
            focal_length,
            0.0,
            (self.width - 1) * 0.5,
            0.0,
            0.0,
            focal_length,
            (self.height - 1) * 0.5,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]

        self.latest_sim_time = None
        self.publisher = self.create_publisher(CameraInfo, self.topic, 10)
        self.create_subscription(ClockMessage, "/clock", self.on_clock, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish)

    def on_clock(self, message: ClockMessage) -> None:
        self.latest_sim_time = message.clock

    def publish(self) -> None:
        self.camera_info.header.stamp = (
            self.latest_sim_time
            if self.latest_sim_time is not None
            else self.get_clock().now().to_msg()
        )
        self.camera_info.header.frame_id = self.frame_id
        self.publisher.publish(self.camera_info)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
