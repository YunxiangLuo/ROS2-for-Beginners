# realsense_lab — 第 12 章 RealSense 相机实验

- 包类型：`ament_python`
- ROS 2 Jazzy + `realsense2_camera`

## 简介

本章练习 RealSense D400 系列深度相机的集成。`realsense_lab` 包提供

`realsense.launch.py`，委托 `realsense2_camera/rs_launch.py` 启动相机并发布

彩色图、深度图与点云；章节根目录的 `realsense_launch.sh` 为等价的交互式

shell 脚本。

## 构建

```bash

cd <robot_sim_demo 工作区>

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select realsense_lab

source install/setup.bash
```

## 运行

```bash
# Launch 方式（推荐）
ros2 launch realsense_lab realsense.launch.py
ros2 launch realsense_lab realsense.launch.py pointcloud:=false

# Shell 脚本方式
cd src/lab_code/ch12_lab && bash realsense_launch.sh
```

启动后发布的话题：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 彩色图像 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 深度图像 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |
| `/camera/color/points` | `sensor_msgs/PointCloud2` | 彩色点云（pointcloud:=true） |

## 依赖安装

```bash

sudo apt install ros-jazzy-realsense2-camera
```

## 测试

```bash
colcon test --packages-select realsense_lab
colcon test-result --all
```

## 运行结果

相机启动后可用 `rqt_image_view /camera/color/image_raw` 查看图像，

截图保存至 `docs/images/realsense_topics.png`（需真实相机硬件）。
