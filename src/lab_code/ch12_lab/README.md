# 第12章 实验代码：多传感器数据采集与融合

本章学习在 ROS2 中集成和使用 RealSense 深度相机。

## 文件说明

| 文件 | 用途 | 运行方式 |
|------|------|----------|
| `realsense_launch.sh` | RealSense 深度相机启动脚本。启动 D400 系列相机节点，发布彩色图像、深度图和点云数据 | `bash realsense_launch.sh` |

## 运行说明

```bash
cd src/lab_code/ch12_lab/
bash realsense_launch.sh
```

### 启动后发布的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 彩色图像 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图像 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 彩色相机参数 |
| `/camera/depth/color/points` | `sensor_msgs/PointCloud2` | 彩色点云 |

### 依赖

需要安装 RealSense ROS2 驱动：

```bash
sudo apt install ros-jazzy-realsense2-camera
```

或在 ROS2 工作空间中源码编译：

```bash
cd ~/ros2_course_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
cd ~/ros2_course_ws
colcon build
```
