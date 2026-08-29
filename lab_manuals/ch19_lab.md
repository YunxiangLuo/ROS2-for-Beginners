# 第19章 实验：相机驱动、标定与视觉检测

## 当前仓库仿真验证：模拟相机、内参与 TF

### 实验目标

使用 `robot_sim_demo` 的 Gazebo 相机完成 Image、CameraInfo、TF 和 OpenCV 输入验证，再接入本实验的颜色、YOLO 或 AR 节点。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_sim_demo gazebo2.launch.py \
  gui:=true rviz:=false drive:=false
```

```bash
ros2 topic info /camera/image_raw
ros2 topic echo /camera/camera_info --once
ros2 run rqt_image_view rqt_image_view /camera/image_raw
ros2 run tf2_ros tf2_echo base_link camera_link
```

### 观察与验收

应能获得 320x180 图像和 CameraInfo，并确认相机 frame。源码：`src/robot_sim_demo/robot_sim_demo/camera_info_publisher.py`、`src/robot_sim_demo/config/gazebo2_bridge.yaml`、`src/lab_code/ch19_lab/vision_detection_lab/`。仿真内参不等于真实镜头标定结果。

> **对应理论章节**：第30章《相机驱动与标定》、第31章《颜色检测与YOLOv8》、第32章《AR标签检测与手眼标定》
> **实验课时**：6课时  
> **实验代码**：`src/lab_code/ch19_lab/vision_detection_lab/`  

## 实验目标
- 掌握USB相机在ROS2中的驱动方法
- 学会使用cv_bridge进行ROS图像与OpenCV图像转换
- 理解相机内参标定原理和流程
- 能够使用OpenCV进行基本的图像处理
- 掌握OpenCV HSV颜色空间和颜色检测方法
- 学会YOLOv8模型的加载和推理
- 实现ROS2中YOLOv8检测结果的发布
- 掌握AprilTag/ArUco标签的检测和位姿估计方法
- 学会在ROS2中发布标签位姿话题
- 理解手眼标定的原理和流程，能够使用easy_handeye完成眼在手外标定

## 实验环境
- ROS 2 Jazzy
- USB摄像头(或笔记本内置摄像头)
- usb_cam功能包
- OpenCV + cv_bridge
- camera_calibration功能包
- YOLOv8 (ultralytics) + Python3 + onnxruntime（19.2节）
- easy_handeye、aruco_ros功能包，打印的ArUco标定板（19.3节）

## 参考代码说明
`src/lab_code/ch19_lab/vision_detection_lab/`（ament_python 包）提供以下程序：

| 程序 | 功能 |
|------|------|
| `usb_cam_node.py` | USB摄像头图像查看器。订阅 `/image_raw` 话题，使用OpenCV显示图像，按ESC退出 |
| `cv_bridge_demo.py` | cv_bridge图像转换演示。将ROS图像转为OpenCV格式，在图像上绘制橙色矩形，再转回ROS图像发布到 `/image_show` |
| `color_detection_node.py` | HSV颜色检测节点。在图像中检测指定颜色的物体区域 |
| `ar_tag_detection_node.py` | AR标签检测节点。检测ArUco标签并估计位姿 |

构建并运行：
```bash
cd <工作区根目录>
colcon build --symlink-install --packages-select vision_detection_lab
source install/setup.bash

# USB相机（真实硬件）
ros2 run vision_detection_lab usb_cam_node

# 或使用仿真相机（robot_sim_demo 提供 /camera/image_raw）
ros2 launch robot_sim_demo gazebo2.launch.py

# 检测节点
ros2 run vision_detection_lab cv_bridge_demo
ros2 run vision_detection_lab color_detection_node
ros2 run vision_detection_lab ar_tag_detection_node
```

验证：
```bash
ros2 topic echo /camera/image_raw --field header --once
rqt_image_view
```

## 19.1 相机驱动与标定

### 19.1.1 安装依赖
```bash
sudo apt install ros-jazzy-usb-cam
sudo apt install ros-jazzy-cv-bridge
sudo apt install ros-jazzy-camera-calibration
sudo apt install ros-jazzy-rqt-image-view

# 验证安装
ros2 pkg list | grep -E "usb_cam|cv_bridge|camera_calibration"
```

### 19.1.2 检测摄像头设备
```bash
# 查看摄像头设备文件
ls /dev/video*

# 查看摄像头详细信息
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats
```

### 19.1.3 启动usb_cam节点
```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p image_width:=640 \
  -p image_height:=480 \
  -p camera_frame_id:=camera_link \
  -p pixel_format:=yuyv

# 或使用launch文件
ros2 launch usb_cam usb_cam-launch.py
```

### 19.1.4 验证图像流
```bash
# 新终端
# 查看话题列表
ros2 topic list

# 查看图像话题信息
ros2 topic info /image_raw

# 查看图像话题类型
ros2 topic type /image_raw

# 使用rqt_image_view查看图像
rqt_image_view /image_raw

# 查看相机信息
ros2 topic echo /camera_info --once
```

### 19.1.5 创建实验功能包
```bash
cd ~/ros2_arm_ws/src
ros2 pkg create ch19_camera --build-type ament_python --dependencies rclpy sensor_msgs cv_bridge
cd ch19_camera
mkdir -p ch19_camera launch
```

### 19.1.6 编写图像订阅和显示节点
创建 `ch19_camera/image_viewer.py`（参考实现：`src/lab_code/ch19_lab/vision_detection_lab/usb_cam_node.py`）:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )
        self.get_logger().info('图像查看器已启动')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(str(e))
            return

        h, w = cv_image.shape[:2]
        fps_text = f'Size: {w}x{h}'
        cv2.putText(cv_image, fps_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow('Camera View', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
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
```

### 19.1.7 编写图像处理节点 (灰度+边缘检测)
创建 `ch19_camera/image_processor.py`（参考实现：`cv_bridge_demo.py` 演示了"ROS图像→OpenCV→处理→ROS图像"的完整回路）:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )
        self.gray_pub = self.create_publisher(Image, '/image_gray', 10)
        self.edge_pub = self.create_publisher(Image, '/image_edges', 10)
        self.get_logger().info('图像处理节点已启动')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(str(e))
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)

        self.gray_pub.publish(self.bridge.cv2_to_imgmsg(gray, 'mono8'))
        edge_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        self.edge_pub.publish(self.bridge.cv2_to_imgmsg(edge_color, 'bgr8'))

        combined = cv2.hconcat([cv_image, blurred])
        cv2.imshow('Original + Blurred', combined)
        cv2.imshow('Edges', edges)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
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
```

### 19.1.8 编写相机信息发布器
创建 `ch19_camera/camera_info_publisher.py`:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        self.pub = self.create_publisher(CameraInfo, '/camera_info', 10)
        self.timer = self.create_timer(1.0, self.publish_info)
        self.get_logger().info('相机信息发布器已启动')

    def publish_info(self):
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.width = 640
        msg.height = 480
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [530.0, 0.0, 320.0, 0.0, 530.0, 240.0, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [530.0, 0.0, 320.0, 0.0, 0.0, 530.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 19.1.9 相机标定
```bash
# 终端1: 启动相机
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p pixel_format:=yuyv

# 终端2: 启动标定
# 准备标定板: 打印棋盘格, 8x6内部角点, 每个方格24.5mm
ros2 run camera_calibration cameracalibrator.py \
  --size 8x6 --square 0.0245 \
  image:=/image_raw camera:=/camera
```

### 19.1.10 标定操作步骤
1. 将标定板放在摄像头视野内
2. 缓慢移动标定板: 上下、左右、前后、倾斜旋转
3. 观察右侧进度条:
   - X: 左右移动
   - Y: 上下移动
   - Size: 前后移动
   - Skew: 倾斜角度
4. 每个进度条变绿后, 点击CALIBRATE
5. 等待计算完成, 查看标定结果
6. 点击SAVE保存标定参数
7. 点击COMMIT将参数写入到camera_info话题

### 19.1.11 查看标定结果
```bash
# 标定结果保存在
ls /tmp/calibrationdata.tar.gz
tar -xzvf /tmp/calibrationdata.tar.gz -C /tmp/calib_result/
cat /tmp/calib_result/ost.yaml
```

### 19.1.12 使用标定结果矫正图像
创建 `ch19_camera/undistort.py`:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml

class ImageUndistorter(Node):
    def __init__(self):
        super().__init__('image_undistorter')
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.new_camera_matrix = None

        self.sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera_info', self.info_callback, 10
        )
        self.undist_pub = self.create_publisher(Image, '/image_undistorted', 10)

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            h, w = msg.height, msg.width
            self.new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix, self.dist_coeffs, (w, h), 1.0, (w, h)
            )
            self.get_logger().info('相机参数已加载')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(str(e))
            return

        h, w = cv_image.shape[:2]
        undistorted = cv2.undistort(
            cv_image, self.camera_matrix, self.dist_coeffs,
            None, self.new_camera_matrix
        )

        combined = np.hstack([cv_image, undistorted])
        cv2.putText(combined, 'Original', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(combined, 'Undistorted', (w + 10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow('Undistortion Comparison', combined)
        cv2.waitKey(1)

        self.undist_pub.publish(
            self.bridge.cv2_to_imgmsg(undistorted, 'bgr8')
        )

    def load_calibration(self, yaml_file):
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)
        self.camera_matrix = np.array(data['camera_matrix']['data']).reshape(3, 3)
        self.dist_coeffs = np.array(data['distortion_coefficients']['data'])
        self.get_logger().info(f'从文件加载标定结果: {yaml_file}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageUndistorter()
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
```

### 19.1.13 配置setup.py并编译运行
```python
entry_points={
    'console_scripts': [
        'image_viewer = ch19_camera.image_viewer:main',
        'image_processor = ch19_camera.image_processor:main',
        'camera_info_publisher = ch19_camera.camera_info_publisher:main',
        'undistort = ch19_camera.undistort:main',
    ],
},
```

```bash
cd ~/ros2_arm_ws
colcon build --packages-select ch19_camera
source install/setup.bash

# 终端1: 启动相机
ros2 run usb_cam usb_cam_node_exe

# 终端2: 运行图像查看器
ros2 run ch19_camera image_viewer

# 终端3: 运行图像处理
ros2 run ch19_camera image_processor

# 查看处理结果
rqt_image_view /image_gray
rqt_image_view /image_edges
```

### 19.1.14 录制和回放图像数据
```bash
# 录制bag文件
ros2 bag record /image_raw /camera_info -o camera_data

# 回放bag文件
ros2 bag play camera_data
```

## 19.2 颜色检测与 YOLO 检测

### 19.2.1 安装依赖
```bash
pip install ultralytics onnxruntime onnx
pip install opencv-python numpy

# 验证
python3 -c "from ultralytics import YOLO; print('YOLO installed')"
```

### 19.2.2 创建实验包
```bash
cd ~/ros2_arm_ws/src
ros2 pkg create ch19_detection --build-type ament_python --dependencies rclpy sensor_msgs std_msgs vision_msgs cv_bridge
cd ch19_detection
mkdir -p ch19_detection
```

### 19.2.3 编写HSV颜色检测节点
创建 `ch19_detection/color_detector.py`（参考实现：`src/lab_code/ch19_lab/vision_detection_lab/color_detection_node.py`）:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.bridge = CvBridge()

        self.declare_parameter('h_min', 35)
        self.declare_parameter('h_max', 85)
        self.declare_parameter('s_min', 50)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 50)
        self.declare_parameter('v_max', 255)

        self.sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )
        self.result_pub = self.create_publisher(Image, '/detection/color_result', 10)
        self.info_pub = self.create_publisher(String, '/detection/color_info', 10)
        self.get_logger().info('颜色检测节点已启动')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(str(e))
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h_min = self.get_parameter('h_min').value
        h_max = self.get_parameter('h_max').value
        s_min = self.get_parameter('s_min').value
        s_max = self.get_parameter('s_max').value
        v_min = self.get_parameter('v_min').value
        v_max = self.get_parameter('v_max').value

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        result = cv_image.copy()
        objects = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 500:
                continue

            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(result, [box], 0, (0, 255, 0), 2)

            center = (int(rect[0][0]), int(rect[0][1]))
            cv2.circle(result, center, 5, (0, 0, 255), -1)

            objects.append({
                'center': (rect[0][0], rect[0][1]),
                'size': (rect[1][0], rect[1][1]),
                'angle': rect[2],
                'area': area
            })

        info = f'Detected {len(objects)} objects'
        cv2.putText(result, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        self.result_pub.publish(self.bridge.cv2_to_imgmsg(result, 'bgr8'))

        info_msg = String()
        info_msg.data = f'count={len(objects)} '
        for i, obj in enumerate(objects):
            info_msg.data += f'obj{i}:({obj["center"][0]:.0f},{obj["center"][1]:.0f}) '
        self.info_pub.publish(info_msg)

        cv2.imshow('Color Detection', result)
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

### 19.2.4 下载YOLOv8模型
```bash
# 下载YOLOv8n预训练模型
python3 -c "
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
print('Model loaded successfully')

# 导出为ONNX格式
model.export(format='onnx', imgsz=640, simplify=True)
print('Export to ONNX complete')
"
```

### 19.2.5 编写YOLOv8检测节点
创建 `ch19_detection/yolo_detector.py`:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import time

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.bridge = CvBridge()

        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.5)

        model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('conf_threshold').value

        self.get_logger().info(f'加载YOLO模型: {model_path}')
        self.model = YOLO(model_path)

        self.sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )
        self.det_pub = self.create_publisher(
            Detection2DArray, '/yolo/detections', 10
        )
        self.annotated_pub = self.create_publisher(
            Image, '/yolo/annotated', 10
        )

        self.last_time = time.time()
        self.frame_count = 0
        self.get_logger().info('YOLO检测节点已启动')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(str(e))
            return

        t0 = time.time()
        results = self.model(cv_image, conf=self.conf_thresh, verbose=False)
        infer_time = time.time() - t0

        det_msg = Detection2DArray()
        det_msg.header = msg.header

        annotated = results[0].plot()

        for r in results:
            for box in r.boxes:
                det = Detection2D()
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                det.bbox.center.position.x = float((x1 + x2) / 2)
                det.bbox.center.position.y = float((y1 + y2) / 2)
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(int(box.cls[0]))
                hyp.hypothesis.score = float(box.conf[0])
                det.results.append(hyp)
                det_msg.detections.append(det)

        self.det_pub.publish(det_msg)
        self.annotated_pub.publish(
            self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        )

        self.frame_count += 1
        elapsed = time.time() - self.last_time
        if elapsed > 5.0:
            fps = self.frame_count / elapsed
            self.get_logger().info(
                f'FPS: {fps:.1f} | 推理: {infer_time*1000:.0f}ms | '
                f'检测: {len(det_msg.detections)} 个'
            )
            self.frame_count = 0
            self.last_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 19.2.6 编写ONNX推理节点(备选)
创建 `ch19_detection/yolo_onnx_node.py`:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import onnxruntime as ort
import numpy as np
import cv2
import time

class YOLOONNXNode(Node):
    def __init__(self):
        super().__init__('yolo_onnx_node')
        self.bridge = CvBridge()

        self.declare_parameter('model_path', 'yolov8n.onnx')
        self.declare_parameter('conf_thresh', 0.5)
        self.declare_parameter('input_size', 640)

        model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('conf_thresh').value
        self.input_size = self.get_parameter('input_size').value

        self.get_logger().info(f'加载ONNX模型: {model_path}')
        self.session = ort.InferenceSession(
            model_path, providers=['CPUExecutionProvider']
        )
        self.input_name = self.session.get_inputs()[0].name

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

        self.sub = self.create_subscription(
            Image, '/image_raw', self.image_cb, 10
        )
        self.det_pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        self.annotated_pub = self.create_publisher(Image, '/yolo/annotated', 10)
        self.get_logger().info('ONNX YOLO节点已启动')

    def image_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        t0 = time.time()
        h, w = cv_image.shape[:2]

        img = cv2.resize(cv_image, (self.input_size, self.input_size))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, axis=0)

        outputs = self.session.run(None, {self.input_name: img})
        output = outputs[0][0].T

        det_msg = Detection2DArray()
        det_msg.header = msg.header

        annotated = cv_image.copy()
        for row in output:
            scores = row[4:84]
            class_id = np.argmax(scores)
            conf = scores[class_id]
            if conf < self.conf_thresh:
                continue

            cx, cy, bw, bh = row[:4]
            x1 = int((cx - bw / 2) * w / self.input_size)
            y1 = int((cy - bh / 2) * h / self.input_size)
            x2 = int((cx + bw / 2) * w / self.input_size)
            y2 = int((cy + bh / 2) * h / self.input_size)

            det = Detection2D()
            det.bbox.center.position.x = float((x1 + x2) / 2)
            det.bbox.center.position.y = float((y1 + y2) / 2)
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(int(class_id))
            hyp.hypothesis.score = float(conf)
            det.results.append(hyp)
            det_msg.detections.append(det)

            cls_name = self.classes[class_id] if class_id < len(self.classes) else 'unknown'
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f'{cls_name} {conf:.2f}'
            cv2.putText(annotated, label, (x1, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        self.det_pub.publish(det_msg)
        self.annotated_pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))

        infer_time = time.time() - t0
        self.get_logger().info(f'检测: {len(det_msg.detections)} 个, 耗时: {infer_time*1000:.0f}ms')

def main(args=None):
    rclpy.init(args=args)
    node = YOLOONNXNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 19.2.7 编写颜色+形状联合检测
创建 `ch19_detection/color_shape_detector.py`:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorShapeDetector(Node):
    def __init__(self):
        super().__init__('color_shape_detector')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )
        self.pub = self.create_publisher(Image, '/detection/color_shape', 10)

        self.color_ranges = {
            'red': [(0, 50, 50), (10, 255, 255)],
            'red2': [(170, 50, 50), (180, 255, 255)],
            'green': [(35, 50, 50), (85, 255, 255)],
            'blue': [(100, 50, 50), (130, 255, 255)],
            'yellow': [(20, 50, 50), (35, 255, 255)],
        }

    def detect_shape(self, contour):
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
        vertices = len(approx)
        if vertices == 3:
            return 'triangle'
        elif vertices == 4:
            x, y, w, h = cv2.boundingRect(contour)
            aspect = w / h
            return 'square' if 0.9 < aspect < 1.1 else 'rectangle'
        elif vertices > 6:
            return 'circle'
        return 'polygon'

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        result = cv_image.copy()

        detected = []
        for color_name, (lower, upper) in self.color_ranges.items():
            if color_name == 'red2':
                continue
            l = np.array(lower)
            u = np.array(upper)
            mask = cv2.inRange(hsv, l, u)

            if color_name == 'red':
                l2 = np.array(self.color_ranges['red2'][0])
                u2 = np.array(self.color_ranges['red2'][1])
                mask2 = cv2.inRange(hsv, l2, u2)
                mask = cv2.bitwise_or(mask, mask2)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < 1000:
                    continue
                shape = self.detect_shape(c)
                M = cv2.moments(c)
                if M['m00'] == 0:
                    continue
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                detected.append((color_name, shape, (cx, cy)))

                color_bgr = {
                    'red': (0, 0, 255),
                    'green': (0, 255, 0),
                    'blue': (255, 0, 0),
                    'yellow': (0, 255, 255),
                }.get(color_name, (255, 255, 255))

                cv2.drawContours(result, [c], -1, color_bgr, 2)
                label = f'{color_name} {shape}'
                cv2.putText(result, label, (cx-20, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)

        info = f'Objects: {len(detected)}'
        cv2.putText(result, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        self.pub.publish(self.bridge.cv2_to_imgmsg(result, 'bgr8'))
        cv2.imshow('Color+Shape Detection', result)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ColorShapeDetector()
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

### 19.2.8 配置setup.py并编译运行
```python
entry_points={
    'console_scripts': [
        'color_detector = ch19_detection.color_detector:main',
        'yolo_detector = ch19_detection.yolo_detector:main',
        'yolo_onnx_node = ch19_detection.yolo_onnx_node:main',
        'color_shape_detector = ch19_detection.color_shape_detector:main',
    ],
},
```

```bash
cd ~/ros2_arm_ws
colcon build --packages-select ch19_detection
source install/setup.bash

# 终端1: 启动相机
ros2 run usb_cam usb_cam_node_exe

# 终端2: 颜色检测
ros2 run ch19_detection color_detector --ros-args -p h_min:=35 -p h_max:=85

# 调整绿色检测参数
ros2 run ch19_detection color_detector --ros-args -p h_min:=100 -p h_max:=124

# 或运行YOLO检测
ros2 run ch19_detection yolo_detector

# 查看YOLO结果
rqt_image_view /yolo/annotated
ros2 topic echo /yolo/detections
```

### 19.2.9 使用rqt_reconfigure调试参数
```bash
sudo apt install ros-jazzy-rqt-reconfigure
ros2 run rqt_reconfigure rqt_reconfigure
```

## 19.3 AR 标签检测与手眼标定

### 19.3.1 安装依赖
```bash
sudo apt install ros-jazzy-cv-bridge ros-jazzy-vision-msgs
sudo apt install ros-jazzy-easy-handeye
sudo apt install ros-jazzy-aruco-ros

# 验证
ros2 pkg list | grep -E "aruco|handeye"
```

### 19.3.2 生成ArUco标签
```bash
# 创建生成标签脚本
python3 -c "
import cv2
import cv2.aruco as aruco
import os

output_dir = os.path.expanduser('~/aruco_markers')
os.makedirs(output_dir, exist_ok=True)

dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

for marker_id in range(1, 6):
    image = aruco.generateImageMarker(dictionary, marker_id, 200)
    filename = os.path.join(output_dir, f'aruco_{marker_id}.png')
    cv2.imwrite(filename, image)
    print(f'生成: {filename}')
"

# 打印标签, 测量实际边长(如0.065m)
```

### 19.3.3 创建实验包
```bash
cd ~/ros2_arm_ws/src
ros2 pkg create ch19_aruco --build-type ament_python --dependencies rclpy sensor_msgs geometry_msgs cv_bridge tf2_ros
cd ch19_aruco
mkdir -p ch19_aruco
```

### 19.3.4 编写ArUco检测节点
创建 `ch19_aruco/aruco_detector.py`（参考实现：`src/lab_code/ch19_lab/vision_detection_lab/ar_tag_detection_node.py`）:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import tf2_ros

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()

        self.declare_parameter('marker_size', 0.065)
        self.declare_parameter('dictionary', 'DICT_6X6_250')

        self.marker_size = self.get_parameter('marker_size').value
        dict_name = self.get_parameter('dictionary').value

        dict_map = {
            'DICT_4X4_50': aruco.DICT_4X4_50,
            'DICT_4X4_100': aruco.DICT_4X4_100,
            'DICT_5X5_50': aruco.DICT_5X5_50,
            'DICT_5X5_100': aruco.DICT_5X5_100,
            'DICT_6X6_50': aruco.DICT_6X6_50,
            'DICT_6X6_250': aruco.DICT_6X6_250,
            'DICT_7X7_50': aruco.DICT_7X7_50,
            'DICT_7X7_100': aruco.DICT_7X7_100,
            'DICT_APRILTAG_16h5': aruco.DICT_APRILTAG_16h5,
            'DICT_APRILTAG_25h9': aruco.DICT_APRILTAG_25h9,
            'DICT_APRILTAG_36h10': aruco.DICT_APRILTAG_36h10,
            'DICT_APRILTAG_36h11': aruco.DICT_APRILTAG_36h11,
        }
        dict_aruco = dict_map.get(dict_name, aruco.DICT_6X6_250)
        dictionary = aruco.getPredefinedDictionary(dict_aruco)
        params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(dictionary, params)

        self.camera_matrix = None
        self.dist_coeffs = None

        self.tf_br = tf2_ros.TransformBroadcaster(self)

        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera_info', self.info_callback, 10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, '/aruco/pose', 10
        )
        self.image_pub = self.create_publisher(
            Image, '/aruco/result', 10
        )
        self.get_logger().info('ArUco检测节点已启动')

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('相机参数已接收')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn('等待相机参数...')
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            for i in range(len(ids)):
                aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs,
                               rvecs[i], tvecs[i], 0.03)

                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'camera_link'
                pose_msg.pose.position.x = float(tvecs[i][0][0])
                pose_msg.pose.position.y = float(tvecs[i][0][1])
                pose_msg.pose.position.z = float(tvecs[i][0][2])

                rmat, _ = cv2.Rodrigues(rvecs[i])
                q = self.rotation_matrix_to_quaternion(rmat)
                pose_msg.pose.orientation.x = q[0]
                pose_msg.pose.orientation.y = q[1]
                pose_msg.pose.orientation.z = q[2]
                pose_msg.pose.orientation.w = q[3]

                marker_id = int(ids[i][0])
                self.pose_pub.publish(pose_msg)

                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'camera_link'
                t.child_frame_id = f'aruco_{marker_id}'
                t.transform.translation.x = float(tvecs[i][0][0])
                t.transform.translation.y = float(tvecs[i][0][1])
                t.transform.translation.z = float(tvecs[i][0][2])
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                self.tf_br.sendTransform(t)

                self.get_logger().info(
                    f'检测到标签 {marker_id}: '
                    f'pos({tvecs[i][0][0]:.3f}, {tvecs[i][0][1]:.3f}, {tvecs[i][0][2]:.3f})'
                )

        cv2.putText(frame, f'Markers: {len(ids) if ids is not None else 0}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        cv2.imshow('ArUco Detection', frame)
        cv2.waitKey(1)

    @staticmethod
    def rotation_matrix_to_quaternion(r):
        q = np.empty(4)
        t = np.trace(r)
        if t > 0:
            s = np.sqrt(1.0 + t) * 2
            q[3] = 0.25 * s
            q[0] = (r[2, 1] - r[1, 2]) / s
            q[1] = (r[0, 2] - r[2, 0]) / s
            q[2] = (r[1, 0] - r[0, 1]) / s
        else:
            i = np.argmax([r[0, 0], r[1, 1], r[2, 2]])
            s = np.sqrt(1.0 + r[i, i]) * 2
            q[3] = (r[2, 1] - r[1, 2]) / s if i == 0 else \
                   (r[0, 2] - r[2, 0]) / s if i == 1 else \
                   (r[1, 0] - r[0, 1]) / s
            q[i] = 0.25 * s
        return q

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
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

### 19.3.5 运行ArUco检测
```bash
# 终端1: 启动相机
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p camera_frame_id:=camera_link

# 终端2: 启动检测 (指定标签实际边长m)
ros2 run ch19_aruco aruco_detector --ros-args -p marker_size:=0.065

# 查看检测结果
rqt_image_view /aruco/result
ros2 topic echo /aruco/pose --once
```

### 19.3.6 坐标变换验证
```bash
# 查看TF树
ros2 run tf2_tools view_frames.py

# 查看坐标变换
ros2 run tf2_ros tf2_echo camera_link aruco_1
```

### 19.3.7 准备手眼标定
1. 在机械臂末端固定ArUco标定板
2. 启动机械臂MoveIt2演示环境（任选其一，见第17章"实验环境"统一说明）:
```bash
# 课程提供的xArm仿真（纯 MoveIt + RViz / 含 Gazebo）
ros2 launch xarm_ros2_arm_only arm_only_move_group.launch.py
# 或 ros2 launch xarm_ros2_arm_only arm_only.launch.py
```

3. 启动相机和ArUco检测:
```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p camera_frame_id:=camera_link
```

### 19.3.8 编写手眼标定launch文件
创建 `launch/handeye_calibration.launch.py`:
```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    marker_size = LaunchConfiguration('marker_size', default='0.094')
    marker_id = LaunchConfiguration('marker_id', default='1')

    return LaunchDescription([
        Node(
            package='easy_handeye',
            executable='calibrate',
            name='handeye_calibration',
            parameters=[{
                'namespace_prefix': 'xarm',
                'move_group': 'xarm_group',
                'eye_on_hand': False,
                'tracking_base_frame': 'camera_link',
                'tracking_marker_frame': 'aruco_marker_frame',
                'robot_base_frame': 'base_link',
                'robot_effector_frame': 'gripper_centor_link',
                'freehand_robot_movement': False,
            }],
        ),
    ])
```

### 19.3.9 执行手眼标定
```bash
# 启动标定GUI
ros2 launch ch19_aruco handeye_calibration.launch.py \
  marker_size:=0.094 marker_id:=1

# 标定步骤:
# 1. 移动机械臂使标定板在相机视野内
# 2. 点击 "Check Starting Pose"
# 3. 点击 "Next Pose" → "Plan" → "Execute" → "Take Sample"
# 4. 重复17次采样
# 5. 点击 "Compute" 计算标定结果
# 6. 点击 "Save" 保存结果
```

### 19.3.10 查看标定结果
```bash
# 标定结果保存在
ls ~/.ros/easy_handeye/
cat ~/.ros/easy_handeye/xarm_handeye_calibration_*.yaml
```

### 19.3.11 使用标定结果发布TF
创建 `ch19_aruco/calibration_tf_publisher.py`:
```python
import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import yaml
import os

class CalibrationTFPublisher(Node):
    def __init__(self):
        super().__init__('calibration_tf_publisher')
        self.br = tf2_ros.StaticTransformBroadcaster(self)

        self.declare_parameter('calibration_file', '')
        calib_file = self.get_parameter('calibration_file').value

        if calib_file and os.path.exists(calib_file):
            self.load_calibration(calib_file)
        else:
            self.use_default_params()

    def load_calibration(self, filepath):
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)

        trans = data.get('transformation', data)
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = trans.get('header', {}).get('frame_id', 'base_link')
        t.child_frame_id = trans.get('child_frame_id', 'camera_link')

        tr = trans.get('transform', {}).get('translation', trans)
        rot = trans.get('transform', {}).get('rotation', trans)

        t.transform.translation.x = float(tr.get('x', 1.0))
        t.transform.translation.y = float(tr.get('y', -0.6))
        t.transform.translation.z = float(tr.get('z', 0.55))
        t.transform.rotation.x = float(rot.get('x', 0.0))
        t.transform.rotation.y = float(rot.get('y', 0.0))
        t.transform.rotation.z = float(rot.get('z', 0.0))
        t.transform.rotation.w = float(rot.get('w', 1.0))

        self.br.sendTransform(t)
        self.get_logger().info(f'从标定文件加载: {filepath}')

    def use_default_params(self):
        self.declare_parameter('x', 1.0)
        self.declare_parameter('y', -0.6)
        self.declare_parameter('z', 0.55)
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        t.transform.translation.x = self.get_parameter('x').value
        t.transform.translation.y = self.get_parameter('y').value
        t.transform.translation.z = self.get_parameter('z').value
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 19.3.12 配置setup.py并编译运行
```python
entry_points={
    'console_scripts': [
        'aruco_detector = ch19_aruco.aruco_detector:main',
        'calibration_tf_publisher = ch19_aruco.calibration_tf_publisher:main',
    ],
},
```

```bash
cd ~/ros2_arm_ws
colcon build --packages-select ch19_aruco
source install/setup.bash

# 发布标定结果TF
ros2 run ch19_aruco calibration_tf_publisher \
  --ros-args -p calibration_file:=~/.ros/easy_handeye/xarm_handeye_calibration_xxx.yaml

# 验证TF
ros2 run tf2_ros tf2_echo base_link camera_link
```

## 实验结果与分析
- usb_cam成功驱动摄像头并发布/image_raw话题
- cv_bridge实现了ROS Image和OpenCV图像格式的双向转换
- Canny边缘检测可以提取图像中的边缘特征
- 相机标定获取了内参矩阵和畸变系数, 可用于图像矫正
- HSV颜色检测通过设定H/S/V范围可以准确提取目标颜色的区域
- YOLOv8模型可以实时检测80类COCO数据集目标
- 颜色检测结合形状分析可以更准确地进行目标识别
- YOLO的推理速度取决于模型大小和硬件配置
- ArUco检测节点成功识别标签并估计6D位姿
- TF广播使得标签坐标系在ROS2系统中可用
- easy_handeye完成了眼在手外标定, 建立了base_link到camera_link的变换关系
- 标定精度受标定板质量和采样姿势多样性影响
- 参考代码 `vision_detection_lab` 的四个节点与本章19.1~19.3的对应练习一致，可在无摄像头时用仿真图像流验证

## 思考题
1. 相机标定中每个进度条(X/Y/Size/Skew)代表什么? 为什么需要多样化的标定板姿态?
2. 畸变矫正对后续视觉任务(如目标检测、位姿估计)有什么影响?
3. BGR8和RGB8格式有什么区别? 在OpenCV中应使用哪种?
4. 如何在没有标定板的情况下估算相机内参?
5. HSV颜色空间相比RGB有什么优势? 为什么颜色检测通常使用HSV?
6. YOLOv8的n/s/m/l/x模型之间有什么区别? 如何选择合适的模型?
7. torch模型和ONNX模型在推理性能上有何差异?
8. 如何训练自定义的YOLOv8模型来检测特定物体?
9. ArUco和AprilTag有什么区别? 各有什么优缺点?
10. 眼在手外(eye-on-hand)和眼在手上(eye-in-hand)标定的区别和适用场景?
11. 标定精度受哪些因素影响? 如何提高标定精度?
12. 在标定过程中, 如何判断样本点是否有效?
