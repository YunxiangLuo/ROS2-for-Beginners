#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import math


class TFBroadcaster(Node):
    """发布 TF2 坐标树: map -> odom -> base_link -> sensor_link"""

    SENSOR_OFFSETS = {
        "lidar_link": (0.0, 0.0, 1.8, 0.0, 0.0, 0.0),
        "imu_link":   (0.5, 0.0, 0.3, 0.0, 0.0, 0.0),
        "gps_link":   (0.8, 0.0, 0.1, 0.0, 0.0, 0.0),
    }

    def __init__(self):
        super().__init__("tf_broadcaster")

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_broadcaster = TransformBroadcaster(self)

        self._publish_static_transforms()

        self.ekf_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.ekf_callback, 10
        )

        self.odom_to_base = TransformStamped()
        self.odom_to_base.header.frame_id = "odom"
        self.odom_to_base.child_frame_id = "base_link"

        self.map_to_odom = TransformStamped()
        self.map_to_odom.header.frame_id = "map"
        self.map_to_odom.child_frame_id = "odom"
        self.map_to_odom.transform.translation.x = 0.0
        self.map_to_odom.transform.translation.y = 0.0
        self.map_to_odom.transform.translation.z = 0.0
        self.map_to_odom.transform.rotation.w = 1.0

        self.get_logger().info("TF Broadcaster 已启动")

    def _publish_static_transforms(self):
        transforms = []
        for name, (x, y, z, roll, pitch, yaw) in self.SENSOR_OFFSETS.items():
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "base_link"
            t.child_frame_id = name
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z

            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)
            t.transform.rotation.w = cr * cp * cy + sr * sp * sy
            t.transform.rotation.x = sr * cp * cy - cr * sp * sy
            t.transform.rotation.y = cr * sp * cy + sr * cp * sy
            t.transform.rotation.z = cr * cp * sy - sr * sp * cy

            transforms.append(t)

        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info(f"已发布 {len(transforms)} 个静态变换")

    def ekf_callback(self, msg: Odometry):
        stamp = self.get_clock().now().to_msg()

        # map -> odom (保持不变, 或根据初始化确定)
        self.map_to_odom.header.stamp = stamp
        self.dynamic_broadcaster.sendTransform(self.map_to_odom)

        # odom -> base_link (来自 EKF 估计)
        self.odom_to_base.header.stamp = stamp
        self.odom_to_base.transform.translation = msg.pose.pose.position
        self.odom_to_base.transform.rotation = msg.pose.pose.orientation
        self.dynamic_broadcaster.sendTransform(self.odom_to_base)


def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
