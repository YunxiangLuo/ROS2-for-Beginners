"""Bottle detector with optional YOLO and deterministic mock boxes."""

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


def boxes_to_detection_array(boxes, header) -> Detection2DArray:
    detections = Detection2DArray()
    detections.header = header
    for x1, y1, x2, y2, confidence in boxes:
        detection = Detection2D()
        detection.header = header
        detection.bbox.center.position.x = float(x1 + x2) / 2.0
        detection.bbox.center.position.y = float(y1 + y2) / 2.0
        detection.bbox.size_x = float(x2 - x1)
        detection.bbox.size_y = float(y2 - y1)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = "bottle"
        hypothesis.hypothesis.score = float(confidence)
        detection.results.append(hypothesis)
        detections.detections.append(detection)
    return detections


class BottleDetector(Node):
    def __init__(self):
        super().__init__("bottle_detector")
        self.declare_parameter("simulation_mode", True)
        self.bridge = CvBridge()
        self.model = None if bool(self.get_parameter("simulation_mode").value) else self.load_model()
        self.publisher = self.create_publisher(
            Detection2DArray, "/bottle_detections", 10
        )
        self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)

    def load_model(self):
        from ultralytics import YOLO

        return YOLO("yolov8n.pt")

    def image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, "bgr8")
        except CvBridgeError as error:
            self.get_logger().error(str(error))
            return
        if self.model is None:
            height, width = image.shape[:2]
            boxes = [
                (
                    width * 0.4,
                    height * 0.25,
                    width * 0.6,
                    height * 0.85,
                    0.95,
                )
            ]
        else:
            boxes = []
            for result in self.model(image, conf=0.3, classes=[39]):
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    boxes.append((x1, y1, x2, y2, float(box.conf[0])))
        self.publisher.publish(boxes_to_detection_array(boxes, message.header))


def main(args=None):
    rclpy.init(args=args)
    node = BottleDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
