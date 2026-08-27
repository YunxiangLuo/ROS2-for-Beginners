#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class UsbCamViewer(Node):
    def __init__(self):
        super().__init__('usb_cam_viewer')

        self.declare_parameter('image_topic', '/image_raw')
        image_topic = self.get_parameter('image_topic').value

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10)

        self.get_logger().info(f'Subscribing to {image_topic}')
        self.get_logger().info('Press ESC in image window to exit')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        cv2.imshow('USB Camera', cv_image)
        key = cv2.waitKey(1)
        if key == 27:
            self.get_logger().info('ESC pressed, shutting down')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCamViewer()
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
