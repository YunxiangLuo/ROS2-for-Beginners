#!/usr/bin/env python3
"""visualize_sensors.py — 多传感器数据可视化

订阅所有传感器话题并显示:
  - RGB/Depth/SemSeg → OpenCV窗口
  - LiDAR → 统计信息
  - RADAR → 目标列表
  - GNSS → 经纬度
  - IMU → 加速度/角速度

用法:
  python3 visualize_sensors.py

依赖:
  rclpy, sensor_msgs, cv_bridge, nav_msgs, OpenCV, numpy
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, Imu
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import struct


class SensorVisualizer(Node):
    def __init__(self):
        super().__init__('sensor_visualizer')
        self.bridge = CvBridge()

        # 图像话题
        self.rgb_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw',
            self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw',
            self.depth_callback, 10)
        self.semseg_sub = self.create_subscription(
            Image, '/camera/semseg/image_raw',
            self.semseg_callback, 10)

        # LiDAR话题
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar/points',
            self.lidar_callback, 10)

        # RADAR话题 (使用泛型订阅)
        self.radar_sub = self.create_subscription(
            LaserScan, '/radar/detections',
            self.radar_callback, 10)

        # GNSS话题
        self.gnss_sub = self.create_subscription(
            NavSatFix, '/gnss/data',
            self.gnss_callback, 10)

        # IMU话题
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data',
            self.imu_callback, 10)

        self.get_logger().info('传感器可视化节点已启动')
        self.get_logger().info('按 q 键退出图像窗口')

    def rgb_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'RGB转换失败: {e}')
            return

        info_text = f'{msg.width}x{msg.height} | {msg.encoding}'
        cv2.putText(cv_img, info_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow('RGB Camera', cv_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def depth_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        except CvBridgeError as e:
            self.get_logger().error(f'Depth转换失败: {e}')
            return

        # 归一化深度图用于显示
        depth_normalized = cv2.normalize(cv_img, None, 0, 255,
                                         cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

        cv2.imshow('Depth Camera', depth_colored)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def semseg_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        except CvBridgeError as e:
            self.get_logger().error(f'SemSeg转换失败: {e}')
            return

        # 语义标签可视化（映射到彩色）
        label_map = {
            0: (0, 0, 0),       # Unlabeled
            1: (128, 128, 128),  # Building
            6: (128, 0, 128),    # Road
            7: (128, 64, 128),   # Sidewalk
            9: (0, 0, 192),      # Vehicle
            4: (220, 20, 60),    # Pedestrian
            8: (0, 128, 0),      # Vegetation
        }

        h, w = cv_img.shape[:2]
        color_img = np.zeros((h, w, 3), dtype=np.uint8)
        for label_id, color in label_map.items():
            mask = cv_img == label_id
            color_img[mask] = color

        cv2.imshow('Semantic Segmentation', color_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def lidar_callback(self, msg):
        points = self.pointcloud2_to_array(msg)
        if len(points) == 0:
            return

        distances = np.sqrt(points[:, 0]**2 +
                            points[:, 1]**2 +
                            points[:, 2]**2)

        self.get_logger().info(
            f'[LiDAR] 点数={len(points)}, '
            f'距离=[{distances.min():.1f}, {distances.max():.1f}]m, '
            f'帧ID={msg.header.frame_id}',
            throttle_duration_sec=1.0
        )

    def radar_callback(self, msg):
        if len(msg.ranges) == 0:
            return

        self.get_logger().info(
            f'[RADAR] 检测目标={len(msg.ranges)}, '
            f'角度范围=[{msg.angle_min:.2f}, {msg.angle_max:.2f}]rad',
            throttle_duration_sec=1.0
        )

        # 打印前5个目标
        for i in range(min(5, len(msg.ranges))):
            angle = msg.angle_min + i * msg.angle_increment
            distance = msg.ranges[i]
            intensity = msg.intensities[i] if msg.intensities else 0
            if distance > 0:
                self.get_logger().info(
                    f'  目标{i+1}: 距离={distance:.2f}m, '
                    f'角度={angle:.3f}rad, 强度={intensity:.2f}',
                    throttle_duration_sec=2.0
                )

    def gnss_callback(self, msg):
        self.get_logger().info(
            f'[GNSS] 纬度={msg.latitude:.6f}, '
            f'经度={msg.longitude:.6f}, '
            f'海拔={msg.altitude:.2f}m',
            throttle_duration_sec=2.0
        )

    def imu_callback(self, msg):
        acc = msg.linear_acceleration
        gyro = msg.angular_velocity
        self.get_logger().info(
            f'[IMU] 加速度=({acc.x:.2f}, {acc.y:.2f}, {acc.z:.2f})m/s², '
            f'角速度=({gyro.x:.3f}, {gyro.y:.3f}, {gyro.z:.3f})rad/s',
            throttle_duration_sec=1.0
        )

    def pointcloud2_to_array(self, cloud_msg):
        """将PointCloud2消息转换为Nx3 numpy数组"""
        fmt_mapping = {
            (PointField.FLOAT32, 4): 'f',
            (PointField.FLOAT64, 8): 'd',
            (PointField.INT32, 4): 'i',
            (PointField.UINT32, 4): 'I',
            (PointField.INT16, 2): 'h',
            (PointField.UINT16, 2): 'H',
            (PointField.INT8, 1): 'b',
            (PointField.UINT8, 1): 'B',
        }

        # 提取x, y, z字段的偏移
        offsets = {}
        for field in cloud_msg.fields:
            if field.name in ('x', 'y', 'z'):
                key = (field.datatype, field.count * 4)
                fmt_char = fmt_mapping.get(key)
                if fmt_char:
                    offsets[field.name] = (field.offset, fmt_char)

        if len(offsets) < 3:
            return np.array([])

        points = []
        for i in range(cloud_msg.width):
            idx = i * cloud_msg.point_step
            point = []
            for axis in ('x', 'y', 'z'):
                offset, fmt = offsets[axis]
                val = struct.unpack_from(fmt, cloud_msg.data, idx + offset)[0]
                point.append(val)
            points.append(point)

        return np.array(points)


def main(args=None):
    rclpy.init(args=args)
    node = SensorVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
