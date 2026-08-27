"""Moving 3D target publisher used by the tracking exercises."""

import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from visualization_msgs.msg import Marker


class Pub3DTarget(Node):
    def __init__(self):
        super().__init__("pub_3d_target")
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("speed", 1.5)
        self.declare_parameter("target_frame", "base_link")
        rate = float(self.get_parameter("rate").value)
        self.speed = float(self.get_parameter("speed").value)
        self.tick = 1.0 / rate

        self.target_publisher = self.create_publisher(PoseStamped, "target_pose", 5)
        self.marker_publisher = self.create_publisher(Marker, "target_marker", 5)
        self.target = PoseStamped()
        self.target.header.frame_id = str(self.get_parameter("target_frame").value)
        self.target.pose.orientation.w = 1.0
        self.marker = Marker()
        self.marker.ns = "target_point"
        self.marker.id = 0
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.lifetime = Duration(seconds=self.tick).to_msg()
        self.marker.scale.x = self.marker.scale.y = self.marker.scale.z = 0.1
        self.marker.color.r = self.marker.color.g = 1.0
        self.marker.color.a = 0.8
        self.theta = 0.0
        self.create_timer(self.tick, self.publish_target)

    def publish_target(self):
        self.target.pose.position.x = 0.42 + 0.1 * abs(math.cos(self.theta))
        self.target.pose.position.z = 0.42 + 0.13 * abs(math.cos(self.theta))
        self.theta += self.speed * self.tick
        stamp = self.get_clock().now().to_msg()
        self.target.header.stamp = stamp
        self.marker.header = self.target.header
        self.marker.pose.position = self.target.pose.position
        self.target_publisher.publish(self.target)
        self.marker_publisher.publish(self.marker)


def main(args=None):
    rclpy.init(args=args)
    node = Pub3DTarget()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
