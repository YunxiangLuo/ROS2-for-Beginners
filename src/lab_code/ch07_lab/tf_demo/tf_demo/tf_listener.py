"""TF2 监听器 — 查询变换并执行坐标点变换"""
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped


class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        if not self.tf_buffer.can_transform(
            'base_link', 'laser_frame', rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=2.0)):
            self.get_logger().warn('等待 laser_frame 变换超时')
            return

        try:
            t = self.tf_buffer.lookup_transform(
                'base_link', 'laser_frame', rclpy.time.Time())
            trans = t.transform.translation
            rot = t.transform.rotation
            self.get_logger().info(
                f'Laser→Base: pos=({trans.x:.3f}, {trans.y:.3f}, {trans.z:.3f}) '
                f'quat=({rot.x:.2f}, {rot.y:.2f}, {rot.z:.2f}, {rot.w:.2f})')
        except LookupException as e:
            self.get_logger().warn(f'变换查询失败: {e}')
            return

        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = 'laser_frame'
        pt.point.x = 1.0
        pt.point.y = 0.5
        pt.point.z = 0.0

        try:
            t_cam = self.tf_buffer.lookup_transform(
                'camera_frame', 'laser_frame', rclpy.time.Time())
            pt_in_camera = do_transform_point(pt, t_cam)
            c = pt_in_camera.point
            self.get_logger().info(
                f'激光点 (1.0,0.5) → 相机系: ({c.x:.3f}, {c.y:.3f}, {c.z:.3f})')
        except LookupException as e:
            self.get_logger().warn(f'坐标变换失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TFListener())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
