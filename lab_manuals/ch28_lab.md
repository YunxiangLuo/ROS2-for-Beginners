# 第28章 实验：交通参与者感知

> **对应理论章节**：第42章《交通参与者感知》  
> **实验课时**：2 课时  
> **实验代码**：`src/lab_code/ch28_lab/`

## 实验目标
- 掌握YOLOv8在CARLA仿真环境中的目标检测与ROS2集成
- 学会LiDAR点云预处理与DBSCAN聚类实现障碍物检测
- 实现基于卡尔曼滤波的多目标跟踪与ID分配

## 实验环境
- ROS 2 Jazzy + CARLA 0.9.16
- carla_ros_bridge 已启动运行
- 传感器：RGB前视相机 + LiDAR
- Python 3.8+：ultralytics, numpy, opencv-python, scikit-learn, filterpy

## 实验前准备

```bash
# 安装依赖
pip install ultralytics scikit-learn filterpy
pip install opencv-python numpy

# 下载YOLOv8模型
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"

# 启动CARLA服务器（终端1）
/path/to/CarlaUE4.sh -quality-level=Low

# 启动ROS2 Bridge（终端2）
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py

# 启动传感器（终端3）
python3 src/lab_code/ch24_lab/sensor_config.py

# 确认话题可用
ros2 topic list | grep -E "(rgb_front|point_cloud)"
```

---

## 练习28.1: CARLA相机YOLO目标检测

**目标**：订阅CARLA前视RGB相机图像，运行YOLOv8推理，发布检测结果。

### 步骤

**1. 创建Python节点**

编写 `yolo_detector.py`，实现：
- 订阅 `/carla/ego_vehicle/rgb_front/image`（sensor_msgs/Image）
- 加载YOLOv8模型进行推理
- 发布 `Detection2DArray` 到 `/perception/yolo/detections`
- 发布标注图像到 `/perception/yolo/visualization`

**2. 关键代码**

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class CarlaYOLODetector(Node):
    def __init__(self):
        super().__init__('carla_yolo_detector')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

        self.sub = self.create_subscription(
            Image, '/carla/ego_vehicle/rgb_front/image',
            self.image_cb, 10
        )
        self.det_pub = self.create_publisher(
            Detection2DArray, '/perception/yolo/detections', 10
        )
        self.vis_pub = self.create_publisher(
            Image, '/perception/yolo/visualization', 10
        )

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame, conf=0.5, verbose=False)[0]

        det_msg = Detection2DArray()
        det_msg.header = msg.header

        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])

            det = Detection2D()
            det.bbox.center.x = float((x1 + x2) / 2)
            det.bbox.center.y = float((y1 + y2) / 2)
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)
            det_msg.detections.append(det)

            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
            cv2.putText(frame, f'{results.names[cls_id]} {conf:.2f}',
                        (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        self.det_pub.publish(det_msg)
        self.vis_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        cv2.imshow('YOLO Detection', frame)
        cv2.waitKey(1)
```

**3. 运行测试**

```bash
python3 src/lab_code/ch28_lab/yolo_detector.py

# 验证话题
ros2 topic echo /perception/yolo/detections
ros2 topic hz /perception/yolo/detections
```

### 预期结果
- 实时显示带标注框的CARLA相机画面
- 检测到车辆、行人、自行车等COCO类别目标
- FPS不低于10帧

---

## 练习28.2: LiDAR点云聚类检测障碍物

**目标**：订阅LiDAR点云，预处理后使用DBSCAN聚类检测障碍物。

### 步骤

**1. 编写LiDAR聚类节点**

创建 `lidar_cluster.py`，实现：
- 订阅 `/carla/ego_vehicle/lidar/point_cloud`（sensor_msgs/PointCloud2）
- 体素滤波降采样（0.1m体素）
- 直通滤波（前方50m范围）
- DBSCAN聚类
- 发布障碍物列表到 `/perception/obstacles`

**2. 关键代码**

```python
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.cluster import DBSCAN

class LidarObstacleDetector(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector')
        self.sub = self.create_subscription(
            PointCloud2, '/carla/ego_vehicle/lidar/point_cloud',
            self.lidar_cb, 10
        )
        self.obstacle_pub = self.create_publisher(
            PointCloud2, '/perception/obstacles', 10
        )

    def preprocess(self, points):
        """点云预处理：直通滤波 + 体素滤波"""
        mask = (points[:, 0] > 0) & (points[:, 0] < 50) & \
               (points[:, 1] > -20) & (points[:, 1] < 20) & \
               (points[:, 2] > -2) & (points[:, 2] < 3)
        points = points[mask]

        # 体素滤波
        voxel_size = 0.1
        voxel_indices = np.floor(points[:, :2] / voxel_size).astype(np.int32)
        _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
        return points[unique_idx]

    def cluster(self, points):
        """DBSCAN聚类"""
        if len(points) < 10:
            return [], np.array([])

        clustering = DBSCAN(eps=0.5, min_samples=10).fit(points[:, :2])
        labels = clustering.labels_
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)

        clusters = []
        for i in range(n_clusters):
            mask = labels == i
            if np.sum(mask) >= 10:
                clusters.append(points[mask])

        return clusters, labels
```

**3. 运行测试**

```bash
python3 src/lab_code/ch28_lab/lidar_cluster.py

# 可视化
ros2 run rviz2 rviz2
# 添加 PointCloud2 显示 /perception/obstacles
```

### 预期结果
- CARLA界面中车辆和行人被正确聚类
- 每个障碍物用不同颜色标记
- 远处密集点云被有效降采样

---

## 练习28.3: 多目标跟踪与可视化

**目标**：结合YOLO检测和LiDAR聚类结果，实现多目标跟踪。

### 步骤

**1. 实现卡尔曼滤波跟踪器**

创建 `object_tracker.py`，包含：
- 卡尔曼滤波器（filterpy库）：状态 = [x, y, vx, vy]
- 匈牙利匹配（scikit-learn的linear_assignment或scipy.optimize）
- 跟踪器管理：创建/更新/删除

**2. 关键代码**

```python
from filterpy.kalman import KalmanFilter
from scipy.optimize import linear_sum_assignment
import numpy as np

class TrackObject:
    def __init__(self, track_id, x, y):
        self.id = track_id
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.F = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]])
        self.kf.H = np.array([[1,0,0,0],[0,1,0,0]])
        self.kf.x = np.array([x, y, 0, 0])
        self.kf.P *= 100
        self.kf.Q = np.eye(4) * 0.1
        self.kf.R = np.eye(2) * 1.0
        self.lost = 0
        self.hits = 1
        self.max_lost = 10

    def predict(self):
        self.kf.predict()
        self.lost += 1
        return self.kf.x[:2]

    def update(self, x, y):
        self.kf.update([x, y])
        self.lost = 0
        self.hits += 1

class TrackerManager:
    def __init__(self):
        self.tracks = {}
        self.next_id = 1

    def update(self, detections):
        # 预测
        for t in self.tracks.values():
            t.predict()

        # 匈牙利匹配
        if not self.tracks or not detections:
            self._handle_unmatched(detections, [])
            return

        cost_matrix = self._compute_cost(detections)
        matched, unmatched_det, unmatched_trk = self._hungarian_match(cost_matrix)

        # 更新匹配的跟踪器
        for i, j in matched:
            self.tracks[list(self.tracks.keys())[j]].update(*detections[i])

        # 创建新跟踪器
        for i in unmatched_det:
            self.tracks[self.next_id] = TrackObject(self.next_id, *detections[i])
            self.next_id += 1

        # 删除丢失的跟踪器
        for j in unmatched_trk:
            tid = list(self.tracks.keys())[j]
            if self.tracks[tid].lost > self.tracks[tid].max_lost:
                del self.tracks[tid]
```

**3. 运行完整系统**

```bash
# 终端1: YOLO检测
python3 src/lab_code/ch28_lab/yolo_detector.py

# 终端2: LiDAR聚类
python3 src/lab_code/ch28_lab/lidar_cluster.py

# 终端3: 目标跟踪
python3 src/lab_code/ch28_lab/object_tracker.py
```

### 预期结果
- 检测目标获得稳定的跟踪ID
- 即使目标短暂被遮挡，跟踪ID保持不变
- 可视化显示每个目标的ID编号和轨迹
- 丢失目标超过阈值后自动删除ID

---

## 实验结果总结

| 练习 | 核心知识点 | 输出话题 | 关键参数 |
|:----:|:----------:|:--------:|:--------:|
| 28.1 | YOLO单阶段检测 | `/perception/yolo/detections` | confidence=0.5 |
| 28.2 | DBSCAN点云聚类 | `/perception/obstacles` | eps=0.5, min_samples=10 |
| 28.3 | 卡尔曼滤波+匈牙利匹配 | `/perception/tracks` | max_lost=10 |

## 思考题

1. YOLO检测中confidence阈值设置多少合适？阈值高低对结果有何影响？
2. DBSCAN的eps参数如何根据LiDAR线束密度进行调节？
3. 卡尔曼滤波的Q和R矩阵如何影响跟踪的平滑性和响应速度？
4. 如何融合YOLO的2D检测结果和LiDAR的3D位置信息？
5. 当多个目标交叉时，匈牙利匹配可能发生ID swap，如何改进？
