#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import yaml
import os


class TF2CameraBroadcaster(Node):
    def __init__(self):
        super().__init__('tf2_camera_broadcaster')

        self.br = tf2_ros.StaticTransformBroadcaster(self)

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'camera_link')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('publish_rate', 50.0)

        self.publish_rate = self.get_parameter('publish_rate').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        calib_file = self.get_parameter('calibration_file').value
        if calib_file and os.path.exists(calib_file):
            self.publish_from_file(calib_file)
        else:
            self.publish_from_params()

        self.get_logger().info(
            f'Publishing TF: {self.parent_frame} → {self.child_frame}')

    def publish_from_file(self, calib_file):
        try:
            with open(calib_file, 'r') as f:
                calib = yaml.safe_load(f)

            t = geometry_msgs.msg.TransformStamped()
            trans = calib['transformation']
            t.header.frame_id = trans['header']['frame_id']
            t.child_frame_id = trans['child_frame_id']

            tr = trans['transform']['translation']
            t.transform.translation.x = tr['x']
            t.transform.translation.y = tr['y']
            t.transform.translation.z = tr['z']

            rot = trans['transform']['rotation']
            t.transform.rotation.x = rot['x']
            t.transform.rotation.y = rot['y']
            t.transform.rotation.z = rot['z']
            t.transform.rotation.w = rot['w']

            t.header.stamp = self.get_clock().now().to_msg()
            self.br.sendTransform(t)
            self.get_logger().info(
                f'Loaded calibration from {calib_file}')
            self.get_logger().info(
                f'Transform: t=({tr["x"]:.4f}, {tr["y"]:.4f}, {tr["z"]:.4f}), '
                f'q=({rot["x"]:.4f}, {rot["y"]:.4f}, '
                f'{rot["z"]:.4f}, {rot["w"]:.4f})')
        except Exception as e:
            self.get_logger().error(f'Failed to load calibration: {e}')

    def publish_from_params(self):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = self.get_parameter('x').value
        t.transform.translation.y = self.get_parameter('y').value
        t.transform.translation.z = self.get_parameter('z').value

        roll = self.get_parameter('roll').value
        pitch = self.get_parameter('pitch').value
        yaw = self.get_parameter('yaw').value

        q = self.euler_to_quaternion(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)
        self.get_logger().info(
            f'Static transform: x={t.transform.translation.x:.4f}, '
            f'y={t.transform.translation.y:.4f}, '
            f'z={t.transform.translation.z:.4f}, '
            f'rpy=({roll:.4f}, {pitch:.4f}, {yaw:.4f})')

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        q = [0.0] * 4
        q[0] = sr * cp * cy - cr * sp * sy
        q[1] = cr * sp * cy + sr * cp * sy
        q[2] = cr * cp * sy - sr * sp * cy
        q[3] = cr * cp * cy + sr * sp * sy
        return q


import math


def main(args=None):
    rclpy.init(args=args)
    node = TF2CameraBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
