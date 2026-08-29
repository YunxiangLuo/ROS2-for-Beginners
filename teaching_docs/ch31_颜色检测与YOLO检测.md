# 第31章 颜色检测与YOLO检测

## 学习目标
- 理解HSV颜色空间及其应用
- 掌握基于颜色阈值的物体检测方法
- 学会集成YOLOv8进行目标检测
- 掌握将检测结果发布为ROS2消息的方法

## 31.1 颜色空间与HSV

### 31.1.1 RGB与HSV颜色空间

RGB（Red-Green-Blue）是最常用的颜色表示方式，但受光照影响大，不适合做颜色检测。

HSV（Hue, Saturation, Value）将颜色分解为色调、饱和度和明度三个维度，更符合人类对颜色的感知，对光照变化不敏感。

- **H（Hue，色调）**：0-180°（OpenCV中归一化为0-180）
  - 红色：0~10和160~180
  - 橙色：11~25
  - 黄色：26~34
  - 绿色：35~77
  - 蓝色：100~124
  - 紫色：125~155

- **S（Saturation，饱和度）**：0-255，值越大颜色越纯
- **V（Value，明度）**：0-255，值越大越亮

### 31.1.2 颜色阈值表

| 颜色 | Hmin | Hmax | Smin | Smax | Vmin | Vmax |
|:----:|:----:|:----:|:----:|:----:|:----:|:----:|
| 红色1 | 0 | 10 | 43 | 255 | 46 | 255 |
| 红色2 | 156 | 180 | 43 | 255 | 46 | 255 |
| 橙色 | 11 | 25 | 43 | 255 | 46 | 255 |
| 黄色 | 26 | 34 | 43 | 255 | 46 | 255 |
| 绿色 | 35 | 77 | 43 | 255 | 46 | 255 |
| 青色 | 78 | 99 | 43 | 255 | 46 | 255 |
| 蓝色 | 100 | 124 | 43 | 255 | 46 | 255 |
| 紫色 | 125 | 155 | 43 | 255 | 46 | 255 |

### 31.1.3 颜色阈值获取工具

```python
#!/usr/bin/env python3
"""HSV颜色阈值调试工具"""
import cv2
import numpy as np

def nothing(x):
    pass

def hsv_threshold_tool():
    """HSV阈值调节GUI工具"""
    cap = cv2.VideoCapture(0)
    cv2.namedWindow('HSV Threshold')

    # 创建滑动条
    cv2.createTrackbar('H_min', 'HSV Threshold', 0, 180, nothing)
    cv2.createTrackbar('H_max', 'HSV Threshold', 180, 180, nothing)
    cv2.createTrackbar('S_min', 'HSV Threshold', 0, 255, nothing)
    cv2.createTrackbar('S_max', 'HSV Threshold', 255, 255, nothing)
    cv2.createTrackbar('V_min', 'HSV Threshold', 0, 255, nothing)
    cv2.createTrackbar('V_max', 'HSV Threshold', 255, 255, nothing)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 读取滑动条值
        h_min = cv2.getTrackbarPos('H_min', 'HSV Threshold')
        h_max = cv2.getTrackbarPos('H_max', 'HSV Threshold')
        s_min = cv2.getTrackbarPos('S_min', 'HSV Threshold')
        s_max = cv2.getTrackbarPos('S_max', 'HSV Threshold')
        v_min = cv2.getTrackbarPos('V_min', 'HSV Threshold')
        v_max = cv2.getTrackbarPos('V_max', 'HSV Threshold')

        # 应用阈值
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)

        # 形态学操作
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 显示结果
        result = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow('Original', frame)
        cv2.imshow('Mask', mask)
        cv2.imshow('Result', result)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            # 保存当前阈值
            print(f'H: [{h_min}, {h_max}], '
                  f'S: [{s_min}, {s_max}], '
                  f'V: [{v_min}, {v_max}]')

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    hsv_threshold_tool()
```

## 31.2 颜色分割与检测

### 31.2.1 颜色检测节点

```python
#!/usr/bin/env python3
"""ROS2颜色检测节点"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        # 可配置参数
        self.declare_parameter('H_min', 35)
        self.declare_parameter('H_max', 77)
        self.declare_parameter('S_min', 43)
        self.declare_parameter('S_max', 255)
        self.declare_parameter('V_min', 46)
        self.declare_parameter('V_max', 255)
        self.declare_parameter('target_color', 'green')

        # 读取参数
        self.H_min = self.get_parameter('H_min').value
        self.H_max = self.get_parameter('H_max').value
        self.S_min = self.get_parameter('S_min').value
        self.S_max = self.get_parameter('S_max').value
        self.V_min = self.get_parameter('V_min').value
        self.V_max = self.get_parameter('V_max').value
        self.target_color = self.get_parameter('target_color').value

        self.bridge = CvBridge()

        # 发布结果图像
        self.result_pub = self.create_publisher(
            Image, '/color_detection/result', 10
        )
        # 发布检测掩码
        self.mask_pub = self.create_publisher(
            Image, '/color_detection/mask', 10
        )

        # 订阅相机图像
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            self.image_callback, 10
        )

        self.get_logger().info(
            f'颜色检测节点已启动，目标颜色: {self.target_color}'
        )

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        self.color_detection(cv_image)

    def color_detection(self, cv_image):
        """执行颜色检测"""
        # 高斯模糊减少噪声
        blurred = cv2.GaussianBlur(cv_image, (7, 7), 0)

        # BGR → HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 创建颜色掩码
        color_low = np.array([self.H_min, self.S_min, self.V_min])
        color_high = np.array([self.H_max, self.S_max, self.V_max])
        mask = cv2.inRange(hsv, color_low, color_high)

        # 形态学操作
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        detection = cv_image.copy()
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:  # 过滤小噪点
                continue

            # 计算最小外接矩形
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # 绘制检测结果
            cv2.drawContours(detection, [box], 0, (0, 255, 0), 2)

            # 绘制中心点
            cx = int(rect[0][0])
            cy = int(rect[0][1])
            cv2.circle(detection, (cx, cy), 4, (0, 0, 255), -1)

            # 添加标签
            cv2.putText(
                detection,
                f'{self.target_color} ({cx}, {cy})',
                (cx - 40, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2
            )

            self.get_logger().info(
                f'检测到{self.target_color}: '
                f'中心({cx}, {cy}), 面积{area:.0f}'
            )

        # 发布结果
        try:
            result_msg = self.bridge.cv2_to_imgmsg(detection, 'bgr8')
            result_msg.header.stamp = self.get_clock().now().to_msg()
            self.result_pub.publish(result_msg)

            mask_msg = self.bridge.cv2_to_imgmsg(mask, 'mono8')
            mask_msg.header.stamp = self.get_clock().now().to_msg()
            self.mask_pub.publish(mask_msg)
        except CvBridgeError as e:
            self.get_logger().error(str(e))

        # 显示
        cv2.imshow('Color Detection', detection)
        cv2.imshow('Mask', mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 31.2.2 多颜色检测

```python
class MultiColorDetector(Node):
    """多颜色同时检测"""

    # 预定义颜色阈值
    COLOR_RANGES = {
        'red': [(np.array([0, 43, 46]), np.array([10, 255, 255])),
                (np.array([156, 43, 46]), np.array([180, 255, 255]))],
        'green': [(np.array([35, 43, 46]), np.array([77, 255, 255]))],
        'blue': [(np.array([100, 43, 46]), np.array([124, 255, 255]))],
        'yellow': [(np.array([26, 43, 46]), np.array([34, 255, 255]))],
        'orange': [(np.array([11, 43, 46]), np.array([25, 255, 255]))],
    }

    COLORS_BGR = {
        'red': (0, 0, 255),
        'green': (0, 255, 0),
        'blue': (255, 0, 0),
        'yellow': (0, 255, 255),
        'orange': (0, 165, 255),
    }

    def __init__(self):
        super().__init__('multi_color_detector')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            self.image_callback, 10
        )
        self.pub = self.create_publisher(
            Image, '/color_detection/multi', 10
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detection = frame.copy()

        for color_name, ranges in self.COLOR_RANGES.items():
            mask = None
            for lower, upper in ranges:
                if mask is None:
                    mask = cv2.inRange(hsv, lower, upper)
                else:
                    mask = cv2.bitwise_or(
                        mask, cv2.inRange(hsv, lower, upper)
                    )

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 300:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(
                    detection, (x, y), (x + w, y + h),
                    self.COLORS_BGR[color_name], 2
                )
                cv2.putText(
                    detection, color_name, (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    self.COLORS_BGR[color_name], 2
                )

        result_msg = self.bridge.cv2_to_imgmsg(detection, 'bgr8')
        result_msg.header = msg.header
        self.pub.publish(result_msg)

        cv2.imshow('Multi Color Detection', detection)
        cv2.waitKey(1)
```

## 31.3 YOLOv8目标检测集成

### 31.3.1 YOLOv8模型架构

YOLO（You Only Look Once）是单阶段目标检测的标杆算法，v8版本进一步优化了速度和精度：

```
网络结构:
  Backbone (C2f模块) → 多尺度特征图
  Head (解耦检测头)  → 分类 + 回归分支

输出张量:
  [batch, 84, 8400]
    │      │    └─ 预测框数 (640/8)^2 * 3 ≈ 8400
    │      └─ 4 (bbox) + 80 (COCO类别)
    └─ 批次大小
```

### 31.3.2 安装依赖

```bash
# 安装ultralytics (YOLOv8)
pip install ultralytics

# 安装onnxruntime (可选的ONNX推理)
pip install onnxruntime

# 下载预训练模型
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
```

### 31.3.3 YOLOv8模型导出

```python
from ultralytics import YOLO

def export_yolo_for_ros2():
    """导出YOLO模型为ONNX格式"""
    model = YOLO('yolov8n.pt')

    model.export(
        format='onnx',
        imgsz=640,
        opset=12,
        half=False,      # FP32精度
        simplify=True,   # 简化模型
        dynamic=False,   # 固定输入尺寸
    )

    print('模型已导出为: yolov8n.onnx')

    # 验证导出的模型
    model_onnx = YOLO('yolov8n.onnx')
    results = model_onnx('test.jpg')
    print(f'检测结果: {len(results[0].boxes)} 个目标')
```

### 31.3.4 ROS2 YOLOv8检测节点

```python
#!/usr/bin/env python3
"""YOLOv8目标检测ROS2节点"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np

class YOLOv8Detector(Node):
    def __init__(self):
        super().__init__('yolov8_detector')
        self.bridge = CvBridge()

        # 参数
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('use_onnx', False)

        model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        use_onnx = self.get_parameter('use_onnx').value

        # 加载YOLO模型
        self.get_logger().info(f'加载YOLO模型: {model_path}')
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            if use_onnx:
                self.model = YOLO(model_path.replace('.pt', '.onnx'))
            self.get_logger().info('YOLO模型加载成功')
        except ImportError:
            self.get_logger().error('ultralytics未安装，请执行: pip install ultralytics')
            self.model = None

        # 发布者
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/yolo/detections', 10
        )
        self.image_pub = self.create_publisher(
            Image, '/yolo/visualization', 10
        )

        # 订阅者
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            self.image_callback, 10
        )

        self.get_logger().info('YOLOv8检测节点已启动')

    def image_callback(self, msg):
        if self.model is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # YOLO推理
        results = self.model(
            frame,
            conf=self.conf_thresh,
            verbose=False
        )[0]

        # 构建Detection2DArray消息
        detections_msg = Detection2DArray()
        detections_msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=msg.header.frame_id
        )

        annotated_frame = frame.copy()

        if results.boxes is not None:
            for box in results.boxes:
                # 提取检测结果
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                confidence = float(box.conf[0])
                class_id = int(box.cls[0])
                class_name = results.names[class_id]

                # 构建Detection2D消息
                detection = Detection2D()
                detection.header = detections_msg.header

                # 设置检测框
                detection.bbox.center.x = float((x1 + x2) / 2)
                detection.bbox.center.y = float((y1 + y2) / 2)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)

                # 设置置信度和类别
                detection.results[0].hypothesis.class_id = class_name
                detection.results[0].hypothesis.score = confidence

                detections_msg.detections.append(detection)

                # 可视化
                color = (0, 255, 0)
                cv2.rectangle(
                    annotated_frame,
                    (int(x1), int(y1)), (int(x2), int(y2)),
                    color, 2
                )
                label = f'{class_name} {confidence:.2f}'
                cv2.putText(
                    annotated_frame, label,
                    (int(x1), int(y1) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
                )

                self.get_logger().info(
                    f'检测到: {class_name} ({confidence:.2f})'
                )

        # 发布检测结果
        self.detection_pub.publish(detections_msg)

        # 发布可视化图像
        vis_msg = self.bridge.cv2_to_imgmsg(annotated_frame, 'bgr8')
        vis_msg.header = msg.header
        self.image_pub.publish(vis_msg)

        cv2.imshow('YOLOv8 Detection', annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 31.3.5 YOLO检测结果后处理

```python
def yolo_postprocess(model_output, conf_thresh=0.5, iou_thresh=0.45):
    """YOLO模型输出后处理（解码 + NMS）"""
    import numpy as np

    # 输入形状: [1, 84, 8400]
    output = model_output[0].T  # [8400, 84]

    boxes = output[:, :4]      # cx, cy, w, h (归一化)
    scores = output[:, 4:]     # 类别概率

    # 获取最高类别
    class_ids = np.argmax(scores, axis=1)
    confidences = np.max(scores, axis=1)

    # 过滤低置信度
    mask = confidences > conf_thresh
    boxes = boxes[mask]
    confidences = confidences[mask]
    class_ids = class_ids[mask]

    # NMS非极大值抑制
    indices = cv2.dnn.NMSBoxes(
        boxes.tolist(), confidences.tolist(),
        conf_thresh, iou_thresh
    )

    results = []
    for i in indices:
        cx, cy, w, h = boxes[i]
        x1 = cx - w / 2
        y1 = cy - h / 2
        results.append({
            'bbox': [x1, y1, w, h],
            'confidence': float(confidences[i]),
            'class_id': int(class_ids[i]),
        })

    return results
```

## 31.4 检测结果发布为ROS2消息

### 31.4.1 使用vision_msgs

ROS2中标准的目标检测消息定义在vision_msgs功能包中：

```bash
# 安装vision_msgs
sudo apt install ros-jazzy-vision-msgs
```

```python
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
    BoundingBox2D
)

def create_detection_msg(
    bbox, confidence, class_name, header
):
    """创建Detection2D消息"""
    detection = Detection2D()
    detection.header = header

    # 设置2D边界框
    detection.bbox.center.x = bbox[0] + bbox[2] / 2
    detection.bbox.center.y = bbox[1] + bbox[3] / 2
    detection.bbox.size_x = bbox[2]
    detection.bbox.size_y = bbox[3]

    # 设置分类结果
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = class_name
    hypothesis.hypothesis.score = confidence
    detection.results = [hypothesis]

    return detection
```

### 34.4.2 自定义检测消息

也可以定义自己的检测消息：

```python
# msg/DetectionResult.msg
std_msgs/Header header
string[] class_names
float32[] scores
float32[] x_min
float32[] y_min
float32[] x_max
float32[] y_max
```

```python
from .msg import DetectionResult

def publish_detection_result(self, detections, header):
    """发布检测结果"""
    msg = DetectionResult()
    msg.header = header

    for det in detections:
        msg.class_names.append(det['class_name'])
        msg.scores.append(float(det['confidence']))
        msg.x_min.append(float(det['bbox'][0]))
        msg.y_min.append(float(det['bbox'][1]))
        msg.x_max.append(float(det['bbox'][2]))
        msg.y_max.append(float(det['bbox'][3]))

    self.detection_pub.publish(msg)
```

### 31.4.3 目标检测与跟踪

```python
class ObjectTracker:
    """简单的目标跟踪器"""

    def __init__(self, max_lost=10):
        self.objects = {}   # class_name → last_bbox
        self.lost_count = {}
        self.max_lost = max_lost

    def update(self, detections):
        """更新跟踪状态"""
        current_ids = set()

        for det in detections:
            class_id = det['class_id']
            current_ids.add(class_id)

            if class_id in self.objects:
                # 更新已有目标
                self.objects[class_id] = det['bbox']
                self.lost_count[class_id] = 0
            else:
                # 新增目标
                self.objects[class_id] = det['bbox']
                self.lost_count[class_id] = 0

        # 标记丢失的目标
        for obj_id in list(self.objects.keys()):
            if obj_id not in current_ids:
                self.lost_count[obj_id] += 1
                if self.lost_count[obj_id] > self.max_lost:
                    del self.objects[obj_id]
                    del self.lost_count[obj_id]

        return self.objects
```

### 31.4.4 发布目标位姿

```python
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

class DetectionToPose(Node):
    """将2D检测转换为3D位姿（假设目标在固定深度）"""
    def __init__(self):
        super().__init__('detection_to_pose')
        self.sub = self.create_subscription(
            Detection2DArray, '/yolo/detections',
            self.detection_callback, 10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, '/target_pose', 10
        )

        # 相机内参
        self.fx = 600.0
        self.fy = 600.0
        self.cx = 320.0
        self.cy = 240.0
        self.depth = 0.5  # 假设的深度值

    def detection_callback(self, msg):
        for detection in msg.detections:
            # 2D像素坐标 → 3D相机坐标
            center_x = detection.bbox.center.x
            center_y = detection.bbox.center.y

            # PnP投影
            x_3d = (center_x - self.cx) * self.depth / self.fx
            y_3d = (center_y - self.cy) * self.depth / self.fy
            z_3d = self.depth

            # 发布位姿
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x_3d
            pose.pose.position.y = y_3d
            pose.pose.position.z = z_3d
            pose.pose.orientation.w = 1.0

            self.pose_pub.publish(pose)
            self.get_logger().info(
                f'目标3D位姿: ({x_3d:.2f}, {y_3d:.2f}, {z_3d:.2f})'
            )
```

## 课后练习

1. 使用HSV颜色空间编写程序检测红色和蓝色物体，并用不同颜色的框标注检测结果。

2. 编写ROS2节点，订阅相机图像，对图像进行颜色检测并发布检测结果mask话题。

3. 安装YOLOv8并编写ROS2检测节点，订阅相机图像，检测COCO数据集的80类物体。

4. 编写程序将YOLO检测结果（类别、置信度、边界框）发布为vision_msgs/Detection2DArray消息。

5. 结合颜色检测和YOLO检测，实现一个融合检测节点：先用颜色检测找到感兴趣区域，再用YOLO在该区域进行精确检测。

---

## 仿真结合实例（当前仓库）：用 Gazebo 相机验证颜色/YOLO 输入链路

### 目标与知识点对应

让颜色分割或 YOLO 节点直接订阅 `robot_sim_demo` 的模拟相机，验证 RGB 图像、CameraInfo 和检测结果话题的连接。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_sim_demo gazebo2.launch.py \
  gui:=true rviz:=false drive:=false
```

```bash
ros2 topic echo /camera/image_raw --once
ros2 topic echo /camera/camera_info --once
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

把本章颜色检测节点的输入设为 `/camera/image_raw`；如果已准备 YOLO 模型，再用 `vision_msgs/Detection2DArray` 检查输出。

### 观察结果与边界

相机画面可作为 HSV 阈值或神经网络的输入；实际检测框、类别和置信度取决于场景、模型文件和节点实现。当前仓库不提供已训练的试剂瓶模型。

### 源码

- 相机：`src/robot_sim_demo/robot_sim_demo/camera_info_publisher.py`
- 图像桥：`src/robot_sim_demo/config/gazebo2_bridge.yaml`
- 视觉实验参考：`src/lab_code/ch19_lab/vision_detection_lab/`
