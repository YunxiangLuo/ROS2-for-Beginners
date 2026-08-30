# 第24章 实验: 多传感器套件与数据采集

> **对应理论章节**：第38章《多传感器套件与数据采集》  
> **实验课时**：3 课时  
> **实验代码**：`src/lab_code/ch24_lab/`

## 实验目标
- 掌握在CARLA中配置和挂载多种传感器到自车
- 学会可视化RGB、Depth、LiDAR等传感器数据
- 掌握使用rosbag录制和回放多传感器数据

## 实验环境
- ROS 2 Jazzy + CARLA 0.9.16
- carla_ros_bridge
- rviz2
- rosbag2

## 练习24.1: 配置并启动多传感器套件

开始前加载课程环境：

```bash
source ~/.config/ros2-course/env.bash
```

### 步骤1: 启动CARLA服务端
```bash
# 终端1: 启动CARLA
carla-server

# 或者使用Docker
docker run --rm -it --gpus all \
  -p 2000-2002:2000-2002 \
  carlasim/carla:0.9.16 \
  ./CarlaUE4.sh -quality-level=Low
```

### 步骤2: 启动carla_ros_bridge
```bash
# 终端2: 启动ROS2桥接
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py \
  host:=localhost port:=2000
```

### 步骤3: 启动自车并挂载传感器
```bash
# 终端3: 启动自车
ros2 launch carla_spawn_objects carla_spawn_objects.launch.py \
  control_role:='ego_vehicle' \
  spwan_point:=robo1 \
  spawn_sensor_only:=false

# 或使用Python脚本手动配置传感器
cd src/lab_code/ch24_lab/
python3 sensor_config.py
```

### 步骤4: 验证传感器话题
```bash
# 查看所有话题
ros2 topic list

# 查看话题信息
ros2 topic info /camera/rgb/image_raw
ros2 topic info /lidar/points
ros2 topic info /imu/data

# 查看话题频率
ros2 topic hz /camera/rgb/image_raw
ros2 topic hz /lidar/points
ros2 topic hz /imu/data
```

### 传感器配置文件参考

`sensor_config.py`定义了以下传感器配置：

| 传感器 | Blueprint | 挂载位置 | 频率 |
|--------|-----------|---------|------|
| 前视RGB相机 | sensor.camera.rgb | (1.6, 0.0, 1.7) | 20Hz |
| Depth相机 | sensor.camera.depth | (1.6, 0.0, 1.7) | 20Hz |
| 64线LiDAR | sensor.lidar.ray_cast | (0.0, 0.0, 2.0) | 10Hz |
| RADAR | sensor.other.radar | (2.0, 0.0, 0.5) | 20Hz |
| GNSS | sensor.other.gnss | (0.0, 0.0, 0.0) | 10Hz |
| IMU | sensor.other.imu | (0.0, 0.0, 0.0) | 50Hz |

## 练习24.2: 可视化各类传感器数据

### 步骤1: 使用rviz2可视化

```bash
# 启动rviz2
rviz2

# 在rviz2中添加显示:
# 1. Image显示 → /camera/rgb/image_raw
# 2. PointCloud2显示 → /lidar/points
# 3. Axes显示 → frame: base_link
```

### 步骤2: 使用rqt_image_view查看图像

```bash
# 查看RGB图像
rqt_image_view /camera/rgb/image_raw

# 查看深度图
rqt_image_view /camera/depth/image_raw
```

### 步骤3: 运行可视化脚本

```bash
# 运行传感器可视化脚本
cd src/lab_code/ch24_lab/
python3 visualize_sensors.py

# 脚本功能:
# - 订阅所有传感器话题
# - OpenCV窗口显示RGB/Depth/SemSeg图像
# - 终端打印LiDAR点云统计信息
# - 终端打印RADAR目标列表
```

### 可视化脚本输出示例

```
=== LiDAR点云统计 ===
点数: 48215
最大距离: 98.3m
最小距离: 0.5m

=== RADAR目标 ===
目标1: 距离=12.5m, 速度=-3.2m/s, 方位角=0.15rad
目标2: 距离=25.1m, 速度=-1.8m/s, 方位角=-0.22rad

=== GNSS数据 ===
纬度: 48.9967°N, 经度: 8.0023°E, 海拔: 45.2m

=== IMU数据 ===
加速度: [0.12, -0.05, 9.81] m/s²
角速度: [0.01, 0.02, 0.00] rad/s
```

## 练习24.3: 录制并回放Rosbag

### 步骤1: 录制传感器数据

```bash
# 方法1: 使用record_bag.sh脚本
cd src/lab_code/ch24_lab/
bash record_bag.sh

# 方法2: 手动录制(推荐)
ros2 bag record \
  /camera/rgb/image_raw \
  /camera/depth/image_raw \
  /camera/semseg/image_raw \
  /lidar/points \
  /radar/detections \
  /gnss/data \
  /imu/data \
  /tf \
  /tf_static \
  -o carla_autonomous_data

# 方法3: 录制所有(不推荐，文件过大)
ros2 bag record -a -o everything
```

### 步骤2: 查看bag信息

```bash
# 查看bag文件信息
ros2 bag info carla_autonomous_data

# 输出示例:
# Files:             carla_autonomous_data_0.db3
# Bag size:          2.4 GiB
# Storage id:        sqlite3
# Duration:          120.5s
# Start:             Jul 18 2026 14:32:10.500
# End:               Jul 18 2026 14:34:11.000
# Messages:          24100
# Topic information:
#   Topic: /camera/rgb/image_raw | Type: sensor_msgs/Image | Count: 2400
#   Topic: /lidar/points         | Type: sensor_msgs/PointCloud2 | Count: 1200
#   Topic: /imu/data             | Type: sensor_msgs/Imu | Count: 6000
#   Topic: /gnss/data            | Type: sensor_msgs/NavSatFix | Count: 1200
```

### 步骤3: 回放bag数据

```bash
# 关闭CARLA服务器和bridge

# 回放bag
ros2 bag play carla_autonomous_data

# 查看回放数据
# 新终端:
rviz2
# 添加Image显示查看回放的RGB图像

# 回放指定速度
ros2 bag play carla_autonomous_data --rate 0.5

# 循环回放(用于算法调试)
ros2 bag play carla_autonomous_data --loop
```

### 步骤4: 从bag中提取数据

```bash
# 使用rosbags工具提取图像
pip install rosbags

# 提取脚本
python3 -c "
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import cv2
import numpy as np

with Reader('carla_autonomous_data') as reader:
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/camera/rgb/image_raw':
            msg = deserialize_cdr(rawdata, connection.msgtype)
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape((msg.height, msg.width, -1))
            cv2.imwrite(f'frame_{timestamp}.png', img)
            break  # 保存第一帧
print('图像提取完成')
"
```

## 实验报告要求

1. 记录各传感器的话题名称、消息类型和频率
2. 截图显示rviz2中RGB图像和LiDAR点云的可视化效果
3. 对比不同LiDAR通道数(16/32/64)的点云密度差异
4. 记录录制bag文件的大小和时长
5. 分析多传感器时间戳对齐的精度

## 常见问题

**Q: 传感器话题没有数据？**
A: 确认CARLA桥接连接正常，检查`ros2 topic echo /carla/status`

**Q: LiDAR点云显示为空？**
A: 检查rviz2的Fixed Frame设置为`map`或`base_link`，确认LiDAR话题有消息输出

**Q: Bag文件过大？**
A: 减少录制时长，或使用压缩格式`ros2 bag record --compression-mode file --compression-format zstd`

**Q: 回放时传感器话题时间戳异常？**
A: 回放时需确认没有其他节点发布相同话题，使用`--clock`参数发布模拟时钟
