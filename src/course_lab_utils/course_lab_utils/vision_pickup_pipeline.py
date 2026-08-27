"""Publish TF-corrected ArUco detections for the pickup server."""

import cv2
import cv2.aruco as aruco
from course_lab_interfaces.msg import MarkerPose, MarkerPoseArray
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
import tf2_geometry_msgs
import tf2_ros

from .vision import ArucoDetectorCompat, detect_aruco_poses


class VisionPickupPipeline(Node):
    def __init__(self):
        super().__init__("vision_pickup_pipeline")
        self.declare_parameter("marker_size", 0.065)
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("display", False)
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.bridge = CvBridge()
        self.detector = ArucoDetectorCompat(aruco.DICT_6X6_250)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_matrix = None
        self.distortion_coefficients = None
        self.latest_markers = MarkerPoseArray()

        self.markers_publisher = self.create_publisher(
            MarkerPoseArray, "/aruco_markers", 10
        )
        self.pose_publisher = self.create_publisher(PoseStamped, "/target_pose", 10)
        self.result_publisher = self.create_publisher(Image, "/pickup_result", 10)
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
        self.create_service(Trigger, "/trigger_pickup", self.status_callback)

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
        marker_array = MarkerPoseArray()
        marker_array.header.stamp = self.get_clock().now().to_msg()
        marker_array.header.frame_id = self.target_frame
        for detection in detections:
            camera_pose = PoseStamped()
            camera_pose.header.stamp = message.header.stamp
            camera_pose.header.frame_id = message.header.frame_id or self.camera_frame
            translation = detection["translation_vector"][0]
            camera_pose.pose.position.x = float(translation[0])
            camera_pose.pose.position.y = float(translation[1])
            camera_pose.pose.position.z = float(translation[2])
            (
                camera_pose.pose.orientation.x,
                camera_pose.pose.orientation.y,
                camera_pose.pose.orientation.z,
                camera_pose.pose.orientation.w,
            ) = detection["quaternion"]
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    camera_pose.header.frame_id,
                    Time(),
                )
                target_pose = tf2_geometry_msgs.do_transform_pose_stamped(
                    camera_pose, transform
                )
            except tf2_ros.TransformException as error:
                self.get_logger().debug(f"TF lookup failed: {error}")
                continue
            marker = MarkerPose()
            marker.id = detection["id"]
            marker.pose = target_pose
            marker_array.markers.append(marker)
            self.pose_publisher.publish(target_pose)

        self.latest_markers = marker_array
        self.markers_publisher.publish(marker_array)
        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(annotated, "bgr8"))
        if bool(self.get_parameter("display").value):
            cv2.imshow("Pickup Pipeline", annotated)
            cv2.waitKey(1)

    def status_callback(self, request, response):
        del request
        response.success = bool(self.latest_markers.markers)
        response.message = f"Detected {len(self.latest_markers.markers)} marker(s)"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VisionPickupPipeline()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
