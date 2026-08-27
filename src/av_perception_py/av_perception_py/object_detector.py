import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Pose2D
from std_msgs.msg import ColorRGBA, Header
import cv_bridge
import cv2
import numpy as np

try:
    from ultralytics import YOLO
    _HAS_YOLO = True
except ImportError:
    _HAS_YOLO = False


class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')

        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('use_cuda', False)

        self.conf_threshold = self.get_parameter('confidence_threshold').value
        model_path = self.get_parameter('model_path').value
        use_cuda = self.get_parameter('use_cuda').value

        self.bridge = cv_bridge.CvBridge()

        self.model = None
        if _HAS_YOLO:
            try:
                self.model = YOLO(model_path)
                if use_cuda:
                    self.model.to('cuda')
                self.get_logger().info(f'YOLO model loaded from {model_path}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load YOLO model: {e}, using color fallback')
                self.model = None
        else:
            self.get_logger().warn('ultralytics not installed, using color-based fallback detection')

        self.sub = self.create_subscription(
            Image, '/carla/ego_vehicle/front_rgb/image', self.image_callback, 10)

        self.det_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detection_markers', 10)

        # Simple color-based detector params
        self.lower_color = np.array([0, 100, 100])
        self.upper_color = np.array([50, 255, 255])

        self.get_logger().info('ObjectDetector node started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        if self.model is not None:
            detections = self._yolo_detect(cv_image)
        else:
            detections = self._color_detect(cv_image)

        det_array = Detection2DArray()
        det_array.header = Header()
        det_array.header.stamp = self.get_clock().now().to_msg()
        det_array.header.frame_id = msg.header.frame_id

        markers = MarkerArray()

        for i, (center_x, center_y, width, height, class_id, confidence) in enumerate(detections):
            detection = Detection2D()
            detection.header = det_array.header
            detection.bbox.center.position.x = float(center_x)
            detection.bbox.center.position.y = float(center_y)
            detection.bbox.center.position.z = 0.0
            detection.bbox.size_x = float(width)
            detection.bbox.size_y = float(height)
            detection.bbox.size_z = 0.0

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(class_id)
            hypothesis.hypothesis.score = float(confidence)
            detection.results.append(hypothesis)

            det_array.detections.append(detection)

            marker = Marker()
            marker.header = det_array.header
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(center_x)
            marker.pose.position.y = float(center_y)
            marker.pose.position.z = 0.0
            marker.scale.x = float(width)
            marker.scale.y = float(height)
            marker.scale.z = 0.1
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)
            marker.lifetime.sec = 1
            markers.markers.append(marker)

        self.det_pub.publish(det_array)
        self.marker_pub.publish(markers)

    def _yolo_detect(self, cv_image):
        results = self.model(cv_image, conf=self.conf_threshold, verbose=False)
        detections = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = box.conf[0].item()
                cls_id = int(box.cls[0].item())
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                w = x2 - x1
                h = y2 - y1
                detections.append((cx, cy, w, h, cls_id, conf))
        return detections

    def _color_detect(self, cv_image):
        detections = []
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w / 2.0
            cy = y + h / 2.0
            detections.append((cx, cy, w, h, 0, 1.0))
        return detections


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
