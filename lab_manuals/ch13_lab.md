# 第13章 实验手册: YOLO + ROS 2 目标检测

## 环境说明
本实验使用 XBot-U 仿真环境 + USB 相机或 Gazebo 模拟相机, 预装 ultralytics 和 onnxruntime。

---

## 练习1: YOLO 模型导出与 ROS 2 推理节点 (~30 分钟)

### 目标
导出 YOLO 模型为 ONNX 格式, 编写 ROS 2 节点实现实时目标检测推理。

### 步骤

#### 1.1 导出 YOLOv8 ONNX 模型
```bash
# 安装 ultralytics
pip install ultralytics onnxruntime

# 导出 ONNX 模型
python3 -c "
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
model.export(format='onnx', opset=12, simplify=True, imgsz=640)
print('模型已导出: yolov8n.onnx')
"
# 验证模型
python3 -c "
import onnx
model = onnx.load('yolov8n.onnx')
onnx.checker.check_model(model)
print('ONNX 模型验证通过')
"
```

#### 1.2 启动相机 (仿真或真实)
```bash
# 选项A: Gazebo 仿真相机
ros2 launch xbot_sim xbot_gazebo_camera.launch.py

# 选项B: 真实 USB 相机
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0

# 验证图像流
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

#### 1.3 编写基础 YOLO ROS 2 推理节点
```python
#!/usr/bin/env python3
"""练习1: YOLO ONNX ROS 2 推理节点"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import onnxruntime as ort
import numpy as np
import cv2
import time


class SimpleYoloNode(Node):
    """基础 YOLO 推理 ROS 2 节点"""
    def __init__(self):
        super().__init__('simple_yolo')

        self.declare_parameter('model_path', 'yolov8n.onnx')
        self.declare_parameter('conf_thresh', 0.5)

        model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('conf_thresh').value

        # 加载 ONNX 模型
        self.get_logger().info(f'加载模型: {model_path}')
        self.session = ort.InferenceSession(
            model_path,
            providers=['CPUExecutionProvider']
        )
        self.input_name = self.session.get_inputs()[0].name
        self.input_size = 640
        self.bridge = CvBridge()

        # 类别映射 (COCO 80 类)
        self.classes = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
            'train', 'truck', 'boat', 'traffic light', 'fire hydrant',
            'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog',
            'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe',
            'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
            'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat',
            'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
            'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
            'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
            'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
            'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
            'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven',
            'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
            'scissors', 'teddy bear', 'hair drier', 'toothbrush',
        ]

        # 订阅 / 发布
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, 10
        )
        self.det_pub = self.create_publisher(
            Detection2DArray, '/yolo/detections', 10
        )
        self.image_pub = self.create_publisher(
            Image, '/yolo/annotated', 10
        )

        # 帧率统计
        self.last_time = time.time()
        self.frame_count = 0

        self.get_logger().info('YOLO 推理节点已就绪')

    def image_cb(self, msg: Image):
        """处理每一帧图像"""
        # 计时开始
        t0 = time.time()

        # 转换
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        # 推理
        h, w = cv_image.shape[:2]
        detections = self.infer(cv_image, w, h)
        infer_time = time.time() - t0

        # 发布检测结果
        det_msg = self.build_msg(detections, msg.header, w, h)
        self.det_pub.publish(det_msg)

        # 发布标注图像
        annotated = self.draw_boxes(cv_image, detections)
        ann_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        ann_msg.header = msg.header
        self.image_pub.publish(ann_msg)

        # 帧率统计
        self.frame_count += 1
        elapsed = time.time() - self.last_time
        if elapsed > 5.0:
            fps = self.frame_count / elapsed
            self.get_logger().info(
                f'FPS: {fps:.1f} | 推理: {infer_time*1000:.0f}ms | '
                f'检测: {len(detections)} 个'
            )
            self.frame_count = 0
            self.last_time = time.time()

    def infer(self, image: np.ndarray, orig_w: int, orig_h: int) -> list:
        """YOLO ONNX 推理"""
        # 预处理
        img = cv2.resize(image, (self.input_size, self.input_size))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, axis=0)

        # ONNX 推理
        outputs = self.session.run(None, {self.input_name: img})
        output = outputs[0][0].T

        # 解析结果
        detections = []
        for row in output:
            scores = row[4:84]
            class_id = np.argmax(scores)
            conf = scores[class_id]
            if conf < self.conf_thresh:
                continue

            cx, cy, bw, bh = row[:4]
            x1 = int((cx - bw / 2) * orig_w / self.input_size)
            y1 = int((cy - bh / 2) * orig_h / self.input_size)
            x2 = int((cx + bw / 2) * orig_w / self.input_size)
            y2 = int((cy + bh / 2) * orig_h / self.input_size)

            detections.append({
                'class_id': int(class_id),
                'class_name': self.classes[class_id] if class_id < len(self.classes) else 'unknown',
                'confidence': float(conf),
                'bbox': (x1, y1, x2, y2),
                'center': ((x1 + x2) / 2, (y1 + y2) / 2),
            })

        return detections

    def build_msg(self, detections: list, header, img_w: int, img_h: int) -> Detection2DArray:
        """构建标准检测消息"""
        msg = Detection2DArray()
        msg.header = header

        for det in detections:
            d = Detection2D()
            x1, y1, x2, y2 = det['bbox']
            d.bbox.center.position.x = float((x1 + x2) / 2)
            d.bbox.center.position.y = float((y1 + y2) / 2)
            d.bbox.size_x = float(x2 - x1)
            d.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(det['class_id'])
            hyp.hypothesis.score = det['confidence']
            d.results.append(hyp)
            msg.detections.append(d)

        return msg

    def draw_boxes(self, image: np.ndarray, detections: list) -> np.ndarray:
        """绘制检测框"""
        result = image.copy()
        colors = [
            (0, 255, 0), (255, 0, 0), (0, 0, 255),
            (255, 255, 0), (255, 0, 255), (0, 255, 255),
        ]
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            color = colors[det['class_id'] % len(colors)]
            label = f"{det['class_name']} {det['confidence']:.2f}"
            cv2.rectangle(result, (x1, y1), (x2, y2), color, 2)
            cv2.putText(result, label, (x1, max(y1 - 5, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        return result


def main():
    rclpy.init()
    node = SimpleYoloNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 1.4 运行与验证
```bash
# 运行 YOLO 节点
python3 lab13_exercise1.py

# 查看检测结果
ros2 topic echo /yolo/detections --field detections --once

# 可视化标注图像
ros2 run rqt_image_view rqt_image_view /yolo/annotated
```

### 验收标准
- [ ] ONNX 模型成功加载, 推理无报错
- [ ] 实时输出检测数量和类别
- [ ] `/yolo/annotated` 话题可显示标注框
- [ ] FPS > 10 (CPU) 或 > 30 (GPU)

---

## 练习2: 检测结果可视化与话题发布 (~30 分钟)

### 目标
发布检测结果为 ROS 2 标准消息和 Marker, 实现多下游节点订阅。

### 步骤

#### 2.1 增强版检测可视化节点
```python
#!/usr/bin/env python3
"""练习2: 检测结果可视化与多话题发布"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import onnxruntime as ort
import numpy as np
import cv2
import json


class VisualizedYoloNode(Node):
    """增强版 YOLO: 多话题发布 + Marker 可视化"""
    def __init__(self):
        super().__init__('visualized_yolo')

        self.declare_parameter('model_path', 'yolov8n.onnx')
        self.declare_parameter('conf_thresh', 0.5)
        self.declare_parameter('classes_of_interest', ['person', 'bottle', 'chair'])

        model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('conf_thresh').value
        self.interest_classes = self.get_parameter('classes_of_interest').value

        self.session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        self.input_size = 640
        self.bridge = CvBridge()

        self.classes = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
                        'train', 'truck', 'boat', 'traffic light', 'fire hydrant',
                        'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog',
                        'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe',
                        'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
                        'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat',
                        'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
                        'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
                        'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
                        'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
                        'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
                        'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven',
                        'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
                        'scissors', 'teddy bear', 'hair drier', 'toothbrush']

        # 订阅
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_cb, 10)
        self.camera_info = None

        # 发布
        self.det_pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        self.image_pub = self.create_publisher(Image, '/yolo/annotated', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/yolo/markers', 10)
        self.text_pub = self.create_publisher(Marker, '/yolo/text_markers', 10)

        # 每个类别单独的话题
        self.per_class_pubs = {}
        for cls in self.interest_classes:
            self.per_class_pubs[cls] = self.create_publisher(
                Detection2DArray, f'/yolo/{cls}', 10
            )

        self.get_logger().info('可视化 YOLO 节点已就绪')

    def camera_info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def image_cb(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        h, w = cv_image.shape[:2]
        detections = self.infer(cv_image, w, h)

        # 发布全景检测
        det_msg = self.build_det_msg(detections, msg.header)
        self.det_pub.publish(det_msg)

        # 发布标注图像
        ann = self.draw_full_annotation(cv_image, detections)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(ann, 'bgr8'))

        # 发布 3D Marker
        self.publish_markers(detections, msg.header, w, h)

        # 按类别分发
        self.publish_per_class(detections, msg.header)

    def publish_markers(self, detections: list, header, img_w: int, img_h: int):
        """发布 3D Marker 数组 (在 rviz 中显示)"""
        marker_array = MarkerArray()
        text_marker = Marker()
        text_marker.header = header
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.scale.z = 0.15
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0
        text_marker.ns = 'yolo_detections'

        for i, det in enumerate(detections):
            # 为每个检测创建球形 Marker (简化表示)
            m = Marker()
            m.header = header
            m.ns = 'yolo_objects'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            # 近似 3D 位置 (假设在相机前方固定距离)
            m.pose.position.x = 2.0
            m.pose.position.y = (det['bbox'][0] / img_w - 0.5) * 3.0
            m.pose.position.z = (0.5 - det['bbox'][1] / img_h) * 2.0
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.8
            marker_array.markers.append(m)

            # 组合文字标记
            text_lines = f"{det['class_name']}\n{det['confidence']:.2f}"
            text_marker.text += f"[{i}] {text_lines}\n"

        self.marker_pub.publish(marker_array)
        self.text_pub.publish(text_marker)

    def publish_per_class(self, detections: list, header):
        """按类别分组发布"""
        class_groups = {}
        for det in detections:
            cls = det['class_name']
            if cls not in class_groups:
                class_groups[cls] = []
            class_groups[cls].append(det)

        for cls, dets in class_groups.items():
            if cls in self.per_class_pubs:
                msg = self.build_det_msg(dets, header)
                self.per_class_pubs[cls].publish(msg)

    def draw_full_annotation(self, image: np.ndarray, detections: list) -> np.ndarray:
        """绘制完整标注 (含统计信息)"""
        result = image.copy()
        colors = {
            'person': (0, 255, 0),
            'bottle': (255, 0, 0),
            'chair': (0, 0, 255),
        }
        # 绘制检测框
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            color = colors.get(det['class_name'], (128, 128, 128))
            cv2.rectangle(result, (x1, y1), (x2, y2), color, 2)
            label = f"{det['class_name']} {det['confidence']:.2f}"
            cv2.putText(result, label, (x1, max(y1 - 5, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # 顶部统计面板
        h, w = result.shape[:2]
        panel = np.zeros((120, w, 3), dtype=np.uint8)
        panel[:] = (50, 50, 50)

        cv2.putText(panel, f'Detections: {len(detections)}', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        class_counts = {}
        for det in detections:
            name = det['class_name']
            class_counts[name] = class_counts.get(name, 0) + 1

        y = 55
        for name, count in sorted(class_counts.items(), key=lambda x: -x[1])[:6]:
            cv2.putText(panel, f'  {name}: {count}', (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            y += 22

        return np.vstack([panel, result])

    def infer(self, image: np.ndarray, orig_w: int, orig_h: int) -> list:
        img = cv2.resize(image, (self.input_size, self.input_size))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, axis=0)
        outputs = self.session.run(None, {self.input_name: img})
        output = outputs[0][0].T

        detections = []
        for row in output:
            scores = row[4:84]
            class_id = np.argmax(scores)
            conf = scores[class_id]
            if conf < self.conf_thresh:
                continue
            cx, cy, bw, bh = row[:4]
            x1 = int((cx - bw / 2) * orig_w / self.input_size)
            y1 = int((cy - bh / 2) * orig_h / self.input_size)
            x2 = int((cx + bw / 2) * orig_w / self.input_size)
            y2 = int((cy + bh / 2) * orig_h / self.input_size)
            detections.append({
                'class_id': int(class_id),
                'class_name': self.classes[class_id] if class_id < len(self.classes) else 'unknown',
                'confidence': float(conf),
                'bbox': (x1, y1, x2, y2),
            })
        return detections

    def build_det_msg(self, detections: list, header) -> Detection2DArray:
        msg = Detection2DArray()
        msg.header = header
        for det in detections:
            d = Detection2D()
            x1, y1, x2, y2 = det['bbox']
            d.bbox.center.position.x = float((x1 + x2) / 2)
            d.bbox.center.position.y = float((y1 + y2) / 2)
            d.bbox.size_x = float(x2 - x1)
            d.bbox.size_y = float(y2 - y1)
            d.results.append(ObjectHypothesisWithPose(
                hypothesis=ObjectHypothesisWithPose.Hypothesis(
                    class_id=str(det['class_id']),
                    score=det['confidence']
                )
            ))
            msg.detections.append(d)
        return msg


def main():
    rclpy.init()
    node = VisualizedYoloNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 验收标准
- [ ] 按类别分组发布正确
- [ ] Marker 在 rviz 中正常显示
- [ ] 统计面板叠加在图像上
- [ ] 每个类别话题均可独立订阅

---

## 练习3: 目标跟随机器人控制 (~30 分钟)

### 目标
基于 YOLO 检测实现机器人对特定类别 (person) 的视觉跟随。

### 步骤

#### 3.1 编写视觉跟随控制节点
```python
#!/usr/bin/env python3
"""练习3: YOLO 目标跟随控制器"""
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
import numpy as np


class YoloTracker(Node):
    """基于 YOLO 检测的视觉跟随控制器"""
    def __init__(self):
        super().__init__('yolo_tracker')

        self.declare_parameter('target_class', 'person')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('desired_bbox_width', 200)
        self.declare_parameter('search_rotation_speed', 0.8)
        self.declare_parameter('max_linear_speed', 0.4)
        self.declare_parameter('max_angular_speed', 1.5)

        self.target_class = self.get_parameter('target_class').value
        self.img_w = self.get_parameter('image_width').value
        self.img_h = self.get_parameter('image_height').value
        self.desired_width = self.get_parameter('desired_bbox_width').value
        self.search_omega = self.get_parameter('search_rotation_speed').value
        self.max_v = self.get_parameter('max_linear_speed').value
        self.max_w = self.get_parameter('max_angular_speed').value

        # 订阅 YOLO 检测
        self.det_sub = self.create_subscription(
            Detection2DArray, '/yolo/detections', self.det_cb, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_detections = []
        self.lost_counter = 0
        self.lost_threshold = 30  # 丢失帧数阈值

        # 控制循环 (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f'跟随控制器已启动, 目标类别: {self.target_class}')

    def det_cb(self, msg: Detection2DArray):
        self.current_detections = msg.detections

    def find_target(self):
        """从检测结果中找到最大置信度的目标"""
        best = None
        best_conf = 0.0
        for det in self.current_detections:
            if len(det.results) == 0:
                continue
            class_id = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score
            if class_id == self.target_class and score > best_conf:
                best = det
                best_conf = score
        return best

    def control_loop(self):
        """PID 视觉跟随控制"""
        target = self.find_target()
        cmd = Twist()

        if target is None:
            self.lost_counter += 1
            if self.lost_counter > self.lost_threshold:
                # 目标丢失: 旋转搜索
                cmd.angular.z = self.search_omega
                self.get_logger().info('搜索目标...', throttle_duration_sec=2.0)
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        else:
            self.lost_counter = 0

            # 目标中心像素坐标
            cx = target.bbox.center.position.x
            cy = target.bbox.center.position.y
            bbox_w = target.bbox.size_x

            # 1. 角速度控制 (水平对齐)
            center_error = (cx - self.img_w / 2.0) / (self.img_w / 2.0)
            cmd.angular.z = -np.clip(center_error * 2.0, -self.max_w, self.max_w)

            # 2. 线速度控制 (距离调节, 基于 bbox 宽度)
            size_error = (self.desired_width - bbox_w) / self.desired_width
            cmd.linear.x = np.clip(size_error * self.max_v, -self.max_v, self.max_v)

            # 3. 停止条件: 距离合适且已对齐
            if abs(center_error) < 0.05 and abs(size_error) < 0.1:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info('目标已对齐!', throttle_duration_sec=2.0)

            self.get_logger().info(
                f'追踪: bbox=({cx:.0f},{cy:.0f}) w={bbox_w:.0f} '
                f'err=({center_error:.2f},{size_error:.2f}) '
                f'cmd=({cmd.linear.x:.2f},{cmd.angular.z:.2f})',
                throttle_duration_sec=1.0
            )

        self.cmd_pub.publish(cmd)

    def stop(self):
        """安全停止"""
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = YoloTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 3.2 完整测试流程
```bash
# 终端1: 启动仿真 (含机器人和相机)
ros2 launch xbot_sim xbot_full.launch.py world:=open_arena

# 终端2: 启动 YOLO 推理
python3 lab13_exercise2.py

# 终端3: 启动跟随控制
python3 lab13_exercise3.py

# 终端4: 在 Gazebo 中放置 person 模型并移动
# (手动拖拽 human model 或使用模型生成脚本)
```

#### 3.3 添加运动模型生成脚本
```python
#!/usr/bin/env python3
"""辅助: Gazebo 中生成移动的行人模型"""
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, Twist


class HumanSpawner(Node):
    """生成移动行人用于测试跟随"""
    def __init__(self):
        super().__init__('human_spawner')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

    def spawn_walking_human(self, name: str, x: float, y: float):
        """生成行走的行人模型"""
        sdf = f"""<?xml version="1.0"?>
        <sdf version="1.6">
          <model name="{name}">
            <static>false</static>
            <link name="link">
              <collision name="collision">
                <geometry><box><size>0.3 0.3 1.7</size></box></geometry>
              </collision>
              <visual name="visual">
                <geometry><box><size>0.3 0.3 1.7</size></box></geometry>
                <material><ambient>1 0 0 1</ambient></material>
              </visual>
            </link>
          </model>
        </sdf>"""

        req = SpawnEntity.Request()
        req.name = name
        req.xml = sdf
        req.initial_pose = Pose()
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = 0.85

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'行人 {name} 已生成 at ({x:.1f}, {y:.1f})')


def main():
    rclpy.init()
    spawner = HumanSpawner()
    spawner.spawn_walking_human('human1', 2.0, 2.0)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 验收标准
- [ ] 机器人成功锁定目标并开始跟随
- [ ] 目标移动时机器人相应调整线速度和角速度
- [ ] 目标丢失后机器人进入搜索模式
- [ ] 接近目标时自动减速停止
- [ ] 可通过 Ctrl+C 安全停止

---

## 练习 4：YOLO 物体检测 → 视觉跟随控制（约 15 分钟）

### 目标
利用 YOLO 检测 Gazebo 中的物体，计算 bbox 中心偏移，生成 cmd_vel 指令实现视觉跟随。

### 步骤

**步骤1：编写 visual_follow.py**
```python
#!/usr/bin/env python3
"""练习4: YOLO 检测→bbox 偏移→cmd_vel 视觉跟随"""
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist


class VisualFollow(Node):
    def __init__(self):
        super().__init__('visual_follow')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.det_sub = self.create_subscription(
            Detection2DArray, '/yolo/detections', self.det_cb, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.target_det = None

    def det_cb(self, msg):
        for det in msg.detections:
            if det.results and det.results[0].hypothesis.score > 0.5:
                self.target_det = det
                return
        self.target_det = None

    def control_loop(self):
        cmd = Twist()
        if self.target_det is None:
            cmd.angular.z = 0.5
        else:
            cx = self.target_det.bbox.center.position.x
            err = (cx - 320) / 320
            cmd.angular.z = -err * 1.5
            if abs(err) < 0.05:
                cmd.linear.x = 0.2
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(VisualFollow())
    rclpy.shutdown()
```

**步骤2：运行**
```bash
# 终端1：启动 YOLO 推理
python3 lab13_exercise1.py
# 终端2：启动视觉跟随
python3 visual_follow.py
```

**✓ 验证**：在相机视野中移动物体，机器人自动转向跟踪目标。目标丢失后旋转搜索。

### 思考题
1. bbox 中心偏移映射到角速度时，为什么需要归一化（除以图像宽度）？
2. 如果相机视野中有多个目标，应该跟踪哪一个？