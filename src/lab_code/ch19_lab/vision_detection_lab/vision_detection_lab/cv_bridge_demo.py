#!/usr/bin/env python3
"""
cv_bridge_demo.py — cv_bridge 图像转换演示

功能:
  1. 订阅 /camera/color/image_raw 图像话题
  2. 使用 cv_bridge 将 ROS 图像转为 OpenCV 格式
  3. 在图像上绘制橙色矩形
  4. 显示处理后的图像窗口
  5. 将 OpenCV 图像转回 ROS 格式发布到 /image_show

运行:
  ros2 run usb_cam usb_cam_node_exe
  python3 cv_bridge_demo.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class CvBridgeDemo(Node):
    def __init__(self):
        super().__init__('cv_bridge_demo')

        self.declare_parameter('image_topic', '/camera/color/image_raw')

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'image_show', 10)

        image_topic = self.get_parameter('image_topic').value
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        self.get_logger().info(f"Waiting for image topics... subscribing to {image_topic}")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        rows, cols, channels = cv_image.shape

        if cols > 120 and rows > 130:
            color = (0, 140, 255)
            thickness = -1
            cv2.rectangle(cv_image, (30, 30), (90, 100), color, thickness)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        except CvBridgeError as e:
            self.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=args)
    node = CvBridgeDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
