"""ROS image node backed by the compatible ArUco detector."""

import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from .vision import ArucoDetectorCompat, detect_aruco_poses


DICTIONARIES = {
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_4X4_100": aruco.DICT_4X4_100,
    "DICT_5X5_50": aruco.DICT_5X5_50,
    "DICT_5X5_100": aruco.DICT_5X5_100,
    "DICT_6X6_50": aruco.DICT_6X6_50,
    "DICT_6X6_250": aruco.DICT_6X6_250,
    "DICT_7X7_50": aruco.DICT_7X7_50,
    "DICT_7X7_1000": aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL,
}


class ArTagDetection(Node):
    def __init__(self):
        super().__init__("ar_tag_detection")
        self.declare_parameter("marker_size", 0.065)
        self.declare_parameter("dictionary", "DICT_6X6_250")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("display", False)
        dictionary_name = str(self.get_parameter("dictionary").value)
        self.detector = ArucoDetectorCompat(
            DICTIONARIES.get(dictionary_name, aruco.DICT_6X6_250)
        )
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion_coefficients = None
        self.pose_publisher = self.create_publisher(PoseStamped, "/aruco_pose", 10)
        self.result_publisher = self.create_publisher(Image, "/aruco_result", 10)
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.image_callback,
            10,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self.info_callback,
            10,
        )

    def info_callback(self, message):
        self.camera_matrix = np.asarray(message.k, dtype=float).reshape(3, 3)
        self.distortion_coefficients = np.asarray(message.d, dtype=float)

    def image_callback(self, message):
        if self.camera_matrix is None:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, "bgr8")
        except CvBridgeError as error:
            self.get_logger().error(str(error))
            return
        annotated, detections = detect_aruco_poses(
            image,
            self.detector,
            self.marker_size,
            self.camera_matrix,
            self.distortion_coefficients,
        )
        for detection in detections:
            pose = PoseStamped()
            pose.header = message.header
            translation = detection["translation_vector"][0]
            pose.pose.position.x = float(translation[0])
            pose.pose.position.y = float(translation[1])
            pose.pose.position.z = float(translation[2])
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = detection["quaternion"]
            self.pose_publisher.publish(pose)
        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(annotated, "bgr8"))
        if bool(self.get_parameter("display").value):
            cv2.imshow("ArUco Tag Detection", annotated)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ArTagDetection()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
