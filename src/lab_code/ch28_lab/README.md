# 第28章 实验代码：交通参与者感知

本章实验围绕CARLA仿真环境中的交通参与者感知，实现YOLO目标检测、LiDAR点云聚类和多目标跟踪。

## 文件说明

| 文件 | 用途 | 运行方式 |
|:----:|------|:--------:|
| `yolo_detector.py` | YOLOv8目标检测节点，订阅CARLA RGB图像，发布Detection2DArray | `python3 yolo_detector.py` |
| `lidar_cluster.py` | LiDAR点云聚类节点，体素滤波+DBSCAN聚类，发布障碍物点云 | `python3 lidar_cluster.py` |
| `object_tracker.py` | 多目标跟踪节点，卡尔曼滤波+匈牙利匹配，发布跟踪结果 | `python3 object_tracker.py` |

## 运行依赖

- Python 3.8+
- ultralytics (YOLOv8)
- scikit-learn (DBSCAN)
- filterpy (Kalman Filter)
- opencv-python, numpy
- ROS 2 Jazzy + sensor_msgs_py

```bash
pip install ultralytics scikit-learn filterpy opencv-python
pip install sensor-msgs-py  # ROS2包
```

## 快速开始

### 前置条件

确保CARLA服务器和ROS2 Bridge已运行：

```bash
# 终端1: CARLA服务器
/path/to/CarlaUE4.sh -quality-level=Low

# 终端2: ROS2 Bridge
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py

# 终端3: 传感器配置
python3 src/lab_code/ch24_lab/sensor_config.py
```

### 运行感知流水线

```bash
# 终端4: YOLO目标检测
cd src/lab_code/ch28_lab/
python3 yolo_detector.py

# 终端5: LiDAR聚类
python3 lidar_cluster.py

# 终端6: 多目标跟踪
python3 object_tracker.py

# 终端7: RViz可视化
rviz2 -d perception.rviz
```

### 话题列表

| 话题 | 类型 | 说明 |
|:----:|:----:|:-----:|
| `/perception/yolo/detections` | `vision_msgs/Detection2DArray` | YOLO检测结果 |
| `/perception/yolo/visualization` | `sensor_msgs/Image` | 标注图像 |
| `/perception/obstacles` | `sensor_msgs/PointCloud2` | 聚类障碍物点云 |
| `/perception/tracks` | `custom_msgs/TrackArray` | 跟踪目标列表 |

## 参数调节

参见各文件开头的参数配置区域。

---

## 安装与编译

```bash
pip install numpy scikit-learn filterpy ultralytics opencv-python
```

## 运行方法

```bash
python lidar_cluster.py    # LiDAR 点云: 直通滤波+体素降采样+DBSCAN 聚类 (需 ROS2)
python object_tracker.py   # 卡尔曼滤波+匈牙利匹配 多目标跟踪 (需 ROS2)
python yolo_detector.py    # YOLOv8 视觉检测 (需 ROS2 + 权重)
```

## 验证

在 CARLA 与 ROS 2 Jazzy 环境中分别启动 YOLO、LiDAR 聚类和跟踪节点，可在对应话题中观察检测、聚类和轨迹结果。本目录未提供独立离线测试套件。
