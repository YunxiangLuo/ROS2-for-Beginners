# 第24章 实验代码：多传感器套件与数据采集

本章学习在CARLA中配置多传感器套件，采集并可视化RGB图像、LiDAR点云、RADAR目标、GNSS/IMU等传感器数据，并使用rosbag进行数据录制与回放。

## 文件说明

| 文件 | 用途 | 运行方式 |
|------|------|----------|
| `sensor_config.py` | 在CARLA中配置并挂载多传感器到自车，发布各传感器ROS2话题 | `python3 sensor_config.py` |
| `visualize_sensors.py` | 订阅并可视化各类传感器数据（图像、点云、目标列表） | `python3 visualize_sensors.py` |
| `record_bag.sh` | rosbag录制脚本，录制关键传感器话题 | `bash record_bag.sh` |

## 运行说明

### 完整流程

```bash
# 终端1: 启动CARLA服务器
/opt/carla/bin/CarlaUE4.sh -quality-level=Low

# 终端2: 启动carla_ros_bridge
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py host:=localhost port:=2000

# 终端3: 启动传感器配置
cd src/lab_code/ch24_lab/
python3 sensor_config.py

# 终端4: 可视化传感器数据
python3 visualize_sensors.py

# 或录制数据
bash record_bag.sh
```

### sensor_config.py

功能：
1. 连接到CARLA服务器（localhost:2000）
2. 生成自车（Tesla Model 3）并设置控制模式
3. 挂载以下传感器到自车：
   - 前视RGB相机（20Hz, 800x600, FOV=90°）
   - Depth相机（20Hz, 800x600）
   - 64线LiDAR（10Hz, 100m范围）
   - RADAR（20Hz, FOV=60°）
   - GNSS（10Hz）
   - IMU（50Hz）
4. 传感器数据通过CARLA原生侦听回调，桥接自动转发到ROS2话题

### visualize_sensors.py

功能：
1. 订阅RGB/Depth相机话题，使用OpenCV实时显示
2. 订阅LiDAR点云，统计并打印点数/距离范围
3. 订阅RADAR检测，打印目标列表
4. 订阅GNSS/IMU，打印位姿信息
5. 键盘按`q`退出

### record_bag.sh

录制以下话题：
- `/camera/rgb/image_raw` — RGB图像
- `/camera/depth/image_raw` — 深度图
- `/camera/semseg/image_raw` — 语义分割
- `/lidar/points` — LiDAR点云
- `/radar/detections` — RADAR目标
- `/gnss/data` — GNSS定位
- `/imu/data` — IMU数据
- `/tf` 和 `/tf_static` — 坐标变换

## 传感器话题汇总

| 话题 | 类型 | 频率 | 帧ID |
|------|------|------|------|
| `/camera/rgb/image_raw` | sensor_msgs/Image | 20Hz | camera_rgb_link |
| `/camera/depth/image_raw` | sensor_msgs/Image | 20Hz | camera_depth_link |
| `/camera/semseg/image_raw` | sensor_msgs/Image | 20Hz | camera_semseg_link |
| `/lidar/points` | sensor_msgs/PointCloud2 | 10Hz | lidar_link |
| `/radar/detections` | carla_msgs/CarlaRadarDetection | 20Hz | radar_link |
| `/gnss/data` | sensor_msgs/NavSatFix | 10Hz | gnss_link |
| `/imu/data` | sensor_msgs/Imu | 50Hz | imu_link |

---

## 安装与编译

```bash
pip install numpy opencv-python pyyaml
```

## 运行方法

```bash
# 生成传感器配置(CARLA 传感器蓝图参数):
python sensor_config.py --camera --lidar --output sensors.yaml
# 可视化相机/LiDAR 数据(需 ros-bridge):
python visualize_sensors.py
# 录制 rosbag(需 ROS2):
bash record_bag.sh
```

## 运行结果

在 CARLA 与 `carla_ros_bridge` 已启动后，可观察到传感器图像、点云和定位数据，并使用 `record_bag.sh` 录制话题。本目录未提供独立离线测试套件。
