#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import time

class CarlaYOLODetector(Node):
    COCO_CLASSES = [
        'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
        'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
        'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep',
        'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
        'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
        'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
        'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
        'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
        'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
        'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
        'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
        'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
        'scissors', 'teddy bear', 'hair drier', 'toothbrush'
    ]

    def __init__(self):
        super().__init__('carla_yolo_detector')

        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_topic', '/carla/ego_vehicle/rgb_front/image')
        self.declare_parameter('target_classes', ['car', 'person', 'truck', 'bus', 'bicycle', 'motorcycle'])

        model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        image_topic = self.get_parameter('image_topic').value
        self.target_classes = self.get_parameter('target_classes').value

        self.bridge = CvBridge()

        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('YOLO model loaded successfully')

        self.sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        self.det_pub = self.create_publisher(
            Detection2DArray, '/perception/yolo/detections', 10
        )
        self.vis_pub = self.create_publisher(
            Image, '/perception/yolo/visualization', 10
        )

        self.frame_count = 0
        self.fps_start = time.time()
        self.get_logger().info(f'YOLO detector ready on {image_topic}')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        t0 = time.time()
        results = self.model(frame, conf=self.conf_thresh, verbose=False)[0]
        infer_time = time.time() - t0

        det_msg = Detection2DArray()
        det_msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=msg.header.frame_id
        )

        annotated = frame.copy()

        if results.boxes is not None:
            for box in results.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                confidence = float(box.conf[0])
                class_id = int(box.cls[0])
                class_name = results.names[class_id]

                if self.target_classes and class_name not in self.target_classes:
                    continue

                det = Detection2D()
                det.header = det_msg.header
                det.bbox.center.x = float((x1 + x2) / 2)
                det.bbox.center.y = float((y1 + y2) / 2)
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = class_name
                hyp.hypothesis.score = confidence
                det.results.append(hyp)
                det_msg.detections.append(det)

                color = (0, 255, 0)
                if class_name == 'person':
                    color = (0, 255, 255)
                elif class_name in ('car', 'truck', 'bus'):
                    color = (0, 0, 255)

                cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                label = f'{class_name} {confidence:.2f}'
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                cv2.rectangle(annotated, (int(x1), int(y1)-th-6), (int(x1)+tw+4, int(y1)), color, -1)
                cv2.putText(annotated, label, (int(x1)+2, int(y1)-4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        self.det_pub.publish(det_msg)

        vis_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        vis_msg.header = msg.header
        self.vis_pub.publish(vis_msg)

        self.frame_count += 1
        elapsed = time.time() - self.fps_start
        if elapsed >= 5.0:
            fps = self.frame_count / elapsed
            self.get_logger().info(
                f'FPS: {fps:.1f} | Inference: {infer_time*1000:.0f}ms | '
                f'Detections: {len(det_msg.detections)}'
            )
            self.frame_count = 0
            self.fps_start = time.time()

        cv2.imshow('YOLO Detection - CARLA', annotated)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rclpy.shutdown()

    def run_yolo_on_image(self, cv_image):
        results = self.model(cv_image, conf=self.conf_thresh, verbose=False)[0]
        return results


def main(args=None):
    rclpy.init(args=args)
    node = CarlaYOLODetector()
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
