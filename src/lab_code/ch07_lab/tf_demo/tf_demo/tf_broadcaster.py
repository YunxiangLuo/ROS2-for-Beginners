"""TF2 广播器 — 发布 odom→base_link 动态变换 + 多传感器静态变换"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.dynamic_br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)

        self.send_static_transforms()

        self.angle = 0.0
        self.radius = 1.0
        self.timer = self.create_timer(0.05, self.publish_odom_tf)

    def send_static_transforms(self):
        now = self.get_clock().now().to_msg()
        transforms = []

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'
        t.transform.translation.x = 0.2
        t.transform.translation.z = 0.1
        t.transform.rotation.w = 1.0
        transforms.append(t)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_frame'
        t.transform.translation.x = 0.15
        t.transform.translation.z = 0.25
        t.transform.rotation.w = 1.0
        transforms.append(t)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        t.transform.translation.z = 0.05
        t.transform.rotation.w = 1.0
        transforms.append(t)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'left_wheel'
        t.transform.translation.y = 0.15
        t.transform.translation.z = -0.05
        t.transform.rotation.w = 1.0
        transforms.append(t)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'right_wheel'
        t.transform.translation.y = -0.15
        t.transform.translation.z = -0.05
        t.transform.rotation.w = 1.0
        transforms.append(t)

        self.static_br.sendTransform(transforms)
        self.get_logger().info('已发送 5 个静态 TF 变换')

    def publish_odom_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.radius * math.cos(self.angle)
        t.transform.translation.y = self.radius * math.sin(self.angle)
        t.transform.rotation.z = math.sin(self.angle / 2)
        t.transform.rotation.w = math.cos(self.angle / 2)
        self.dynamic_br.sendTransform(t)
        self.angle += 0.02


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TFBroadcaster())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
