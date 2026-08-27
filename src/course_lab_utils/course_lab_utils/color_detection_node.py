"""ROS image node backed by the testable color detector."""

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from .vision import detect_color_regions


class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__("color_detection_node")
        for name, value in (
            ("H_min", 35),
            ("H_max", 77),
            ("S_min", 43),
            ("S_max", 255),
            ("V_min", 46),
            ("V_max", 255),
        ):
            self.declare_parameter(name, value)
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("display", False)
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, "color_result", 10)
        self.mask_publisher = self.create_publisher(Image, "color_mask", 10)
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.image_callback,
            10,
        )

    def image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, "bgr8")
            lower = tuple(int(self.get_parameter(name).value) for name in ("H_min", "S_min", "V_min"))
            upper = tuple(int(self.get_parameter(name).value) for name in ("H_max", "S_max", "V_max"))
            annotated, mask, _ = detect_color_regions(image, lower, upper)
            self.image_publisher.publish(self.bridge.cv2_to_imgmsg(annotated, "bgr8"))
            self.mask_publisher.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
        except CvBridgeError as error:
            self.get_logger().error(str(error))
            return
        if bool(self.get_parameter("display").value):
            cv2.imshow("Color Detection Result", annotated)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
