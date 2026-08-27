# 第38章 多传感器套件与数据采集

---

## 学习目标
- 掌握CARLA相机传感器配置与数据格式
- 理解LiDAR点云生成与参数配置
- 学会RADAR、GNSS、IMU传感器使用
- 理解内参/外参标定与坐标系对齐
- 掌握Rosbag数据采集方法

---

## 传感器总览

| 传感器 | 数据类型 | 频率 | 用途 |
|--------|---------|------|------|
| RGB相机 | Image (BGRA) | 20Hz | 视觉感知 |
| Depth相机 | Image (32FC1) | 20Hz | 深度估计 |
| SemSeg相机 | Image (32FC1) | 20Hz | 语义理解 |
| LiDAR | PointCloud2 | 10-20Hz | 3D感知 |
| RADAR | RadarDetection | 20Hz | 目标检测 |
| GNSS | NavSatFix | 10Hz | 全局定位 |
| IMU | Imu | 50Hz | 位姿估计 |

---

## 38.1 相机传感器

- **RGB相机**: sensor.camera.rgb → BGRA 8-bit
- **Depth相机**: sensor.camera.depth → 32位浮点深度
- **SemSeg相机**: sensor.camera.semantic_segmentation → 32位语义标签

| 参数 | 范围 | 说明 |
|------|------|------|
| image_size_x/y | 任意 | 分辨率 |
| fov | 30-120 | 视场角 |
| sensor_tick | 0.0+ | 采样间隔 |

---

## 相机内参

```
K = [fx   0   cx]
    [0   fy   cy]
    [0    0    1]
```

- fx = W / (2 * tan(fov/2))
- fy = H / (2 * tan(fov/2))
- CARLA针孔模型，默认无畸变
- 可后处理添加径向畸变(k1, k2)和切向畸变(p1, p2)

---

## 38.2 LiDAR传感器

```
sensor.lidar.ray_cast
```

| 配置项 | VLP-16 | OS1-64 | Pandar40P |
|--------|--------|--------|-----------|
| channels | 16 | 64 | 40 |
| range | 100m | 120m | 200m |
| points/sec | 300K | 1310K | 720K |
| upper_fov | +15° | +22.5° | +15° |
| lower_fov | -15° | -22.5° | -15° |

---

## 点云格式

```
LiDAR点云 → N x 4 numpy array
  [x, y, z, intensity]

→ sensor_msgs/PointCloud2
  fields: x, y, z, intensity
  frame_id: lidar_link
```

坐标系转换:
- CARLA: X前 Y右 Z上
- ROS: X前 Y左 Z上
- **Y轴取反**是关键

---

## 38.3 RADAR传感器

- 输出: 目标级检测列表
- 每个检测包含:
  - depth: 距离(m)
  - velocity: 径向速度(m/s)
  - azimuth: 水平方位角(rad)
  - altitude: 垂直仰角(rad)

```python
radar_bp.set_attribute('horizontal_fov', '60')
radar_bp.set_attribute('range', '50')
radar_bp.set_attribute('points_per_second', '2000')
```

---

## 38.3 GNSS & IMU

**GNSS:**
- 输出: 纬度/经度/海拔
- 可配置噪声模型(标准差)
- ROS2话题: sensor_msgs/NavSatFix

**IMU:**
- 输出: 六轴数据
  - 加速度计: 三轴加速度(m/s²)
  - 陀螺仪: 三轴角速度(rad/s)
  - 磁力计: 朝向(四元数)
- ROS2话题: sensor_msgs/Imu
- 典型频率: 50Hz

---

## 38.4 传感器标定

**内参** - 传感器内部参数:
- 相机: K矩阵 + 畸变系数
- LiDAR: 通道角度分布

**外参** - 传感器间空间变换:
- 由TF2管理
- 定义 sensor_link → base_link

```yaml
camera_front_link:
  translation: [0.5, 0.0, 0.2]
  rotation: [0.0, 0.0, 0.0, 1.0]
```

---

## 坐标系对齐

| 轴 | CARLA | ROS | 转换 |
|----|-------|-----|------|
| X | 前 | 前 | 不变 |
| Y | 右 | **左** | 取反 |
| Z | 上 | 上 | 不变 |

Yaw同样取反:
```python
yaw_ros = -yaw_carla
```

---

## CARLA ↔ ROS 桥接架构

```
┌──────────┐   ┌──────────────┐   ┌──────────┐
│ CARLA    │ → │ carla_ros_   │ → │ ROS2     │
│ 传感器   │   │ bridge       │   │ 话题     │
└──────────┘   └──────────────┘   └──────────┘
```

- carla_ros_bridge: 官方ros2桥接
- 自动处理坐标系转换
- 支持所有传感器类型

---

## 38.5 Rosbag数据采集

```bash
ros2 bag record \
  /camera/rgb/image_raw \
  /camera/depth/image_raw \
  /lidar/points \
  /gnss/data \
  /imu/data \
  -o carla_dataset
```

**关键话题:**
| 话题 | 类型 | 频率 |
|------|------|------|
| /camera/rgb/image_raw | Image | 20Hz |
| /lidar/points | PointCloud2 | 10Hz |
| /imu/data | Imu | 50Hz |
| /gnss/data | NavSatFix | 10Hz |

---

## 数据同步策略

**方法1: ApproximateTimeSynchronizer**
```python
sync = ApproximateTimeSynchronizer(
    subs, queue_size=10, slop=0.05)
```

**方法2: 精确时间(exact)**
- 仅用于严格对齐的消息
- 需要相同的时间戳

**方法3: 全局时间戳**
- 使用CARLA world.on_tick()
- 统一的时间基准

---

## 数据集管理

推荐目录结构:
```
carla_dataset/
  seq_0000/
    rgb/      000000.png ...
    depth/    000000.png ...
    lidar/    000000.npy ...
    calib/    intrinsics.yaml
               extrinsics.yaml
  seq_0001/
    ...
```

---

## 本章要点

1. CARLA 7种传感器: 3相机 + LiDAR + RADAR + GNSS + IMU
2. 每个传感器通过blueprint配置参数
3. CARLA↔ROS: Y轴取反是核心
4. sensor_tick控制数据频率
5. TF2管理传感器外参
6. Rosbag是标准数据采集方案
