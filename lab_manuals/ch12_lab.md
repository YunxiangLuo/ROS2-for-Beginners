# 第12章 实验：RealSense 多传感器数据采集与融合

## 当前仓库仿真验证：RealSense RGB-D 话题与点云输出

### 实验目标

启动 RealSense D400 系列相机驱动，确认彩色图、深度图、相机内参与点云话题的发布，并用 `ros2 topic` 与 `rqt_image_view` 检查数据质量。注意：本实验需要真实相机硬件，仓库 Gazebo 仿真（第22章的 `gazebo2.launch.py drive:=true`）只提供单目彩色 `/camera/image_raw`，无法替代深度流与点云输出。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 终端 1：启动相机驱动（默认 640x480x30，带点云与深度对齐）
ros2 launch realsense_lab realsense.launch.py
# 不需要点云时：ros2 launch realsense_lab realsense.launch.py pointcloud:=false
```

```bash
# 终端 2：验证话题与频率
ros2 topic list | grep /camera
ros2 topic hz /camera/color/image_raw
ros2 topic echo /camera/color/camera_info --once
```

### 观察与验收

`/camera/color/image_raw` 与 `/camera/depth/image_rect_raw` 应以约 30Hz 稳定发布，`/camera/color/camera_info` 中能看到 D、K 内参矩阵；pointcloud 模式下还应出现 `/camera/color/points`。源码：`src/lab_code/ch12_lab/realsense_lab/launch/realsense.launch.py`。若 `rs-enumerate-devices` 枚举不到设备，优先检查 USB 线材与 `realsense2_camera` 驱动安装。

## 实际运行证据

本实验依赖真实 RealSense D415/D435 硬件，仓库未预置可回放的运行录制。上机时按包 README 的约定将 `rqt_image_view` 截图保存至 `docs/images/realsense_topics.png`，并同步记录 `ros2 topic hz` 的输出作为验收依据。

> **对应理论章节**：第22章《多传感器融合SLAM》
> **实验课时**：2课时  
> **实验代码**：`src/lab_code/ch12_lab/`（`realsense_lab/` 功能包 + `realsense_launch.sh`）  

## 实验目标
- 理解深度相机的多路数据输出：彩色图、深度图、内参与点云
- 掌握 realsense2_camera 驱动在 ROS 2 Jazzy 下的安装与启动
- 学会用 launch 参数控制点云与深度对齐的开关
- 能用 rqt_image_view、RViz2 可视化 RGB-D 数据
- 了解多传感器融合中时间同步（`enable_sync`）的作用

## 实验环境
- ROS 2 Jazzy + `ros-jazzy-realsense2-camera`
- RealSense D400 系列相机（D415/D435）
- rqt_image_view、RViz2
- ros2 bag（数据集录制，可选）

## 参考代码说明
`src/lab_code/ch12_lab/` 下包含一个 ament_python 功能包与一个交互式启动脚本：

```
src/lab_code/ch12_lab/
├── realsense_lab/                    # ROS2 功能包（ament_python）
│   ├── launch/realsense.launch.py    # 委托 realsense2_camera 的 rs_launch.py
│   ├── test/test_realsense_lab.py    # 包级冒烟测试
│   └── package.xml / setup.py
├── realsense_launch.sh               # 等价的交互式启动脚本
└── README.md
```

- `realsense.launch.py` 将固定档位 `depth_module.depth_profile:=640x480x30`、`rgb_camera.color_profile:=640x480x30`、`enable_sync:=true` 传给 `rs_launch.py`；`pointcloud.enable` 与 `align_depth.enable` 跟随 `pointcloud` 参数（默认 true）。
- `realsense_launch.sh` 依次检查 ROS 2 环境、驱动包、设备枚举（`rs-enumerate-devices`），再按模式启动相机：默认 RGB-D、`pointcloud`（RGB-D + 点云 + 对齐）、`highres`（1280x720x15），启动后自动核对关键话题是否发布。

启动后发布的话题（以包 README 为准）：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 彩色图像 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 深度图像 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |
| `/camera/color/points` | `sensor_msgs/PointCloud2` | 彩色点云（pointcloud:=true） |

## 实验步骤

1. 安装驱动并构建实验包：

```bash
sudo apt install ros-jazzy-realsense2-camera

cd <机器人工作区>       # 即含 src/lab_code 的 ROS 2 工作区
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select realsense_lab
source install/setup.bash
```

2. 连接相机后启动（二选一）：

```bash
# Launch 方式（推荐）
ros2 launch realsense_lab realsense.launch.py

# Shell 脚本方式
cd src/lab_code/ch12_lab && bash realsense_launch.sh   # 默认 RGB-D
# bash realsense_launch.sh pointcloud                  # RGB-D + 点云
# bash realsense_launch.sh highres                     # 1280x720x15 高分辨率
```

3. 检查话题频率与内参：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_rect_raw
ros2 topic echo /camera/color/camera_info --once
```

4. 可视化：

```bash
rqt_image_view /camera/color/image_raw        # 彩色
rqt_image_view /camera/depth/image_rect_raw   # 深度
ros2 run rviz2 rviz2                          # 添加 PointCloud2 显示 /camera/color/points
```

5. （可选）录制数据集供后续章节使用：

```bash
ros2 bag record /camera/color/image_raw /camera/depth/image_rect_raw \
  /camera/color/camera_info /camera/depth/camera_info \
  -o realsense_dataset
```

6. 实验结束按 Ctrl+C 停止驱动；开启点云后驱动会同时做深度对齐，CPU 占用略高属正常现象。

## 实验结果与分析

- 彩色与深度流均应稳定在 30Hz；`camera_info` 的 K 矩阵（fx、fy、cx、cy）与出厂标定一致，是第30章相机标定实验的数据基础。
- `align_depth.enable:=true` 后，点云由对齐到彩色光心的深度图生成，彩色/深度视差带来的边缘错位消失。
- `enable_sync:=true` 让 RGB 与深度帧在驱动层做时间戳同步，对应第22章 22.5.3 节"传感器时间同步"的最小实践。

## 思考题

1. 关闭 `pointcloud:=false` 后哪些话题会消失？驱动内部少做了哪些计算？
2. 深度图 `image_rect_raw` 与点云 `PointCloud2` 各适合什么下游算法？
3. 为什么点云需要"深度对齐到彩色"？（提示：彩色与深度镜头光心不同）
4. 结合 22.5.3 节，说明驱动内 `enable_sync` 与消息过滤器 `ApproximateTimeSynchronizer` 的关系。
