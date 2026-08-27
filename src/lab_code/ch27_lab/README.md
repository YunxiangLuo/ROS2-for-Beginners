# 第27章 实验代码：多传感器融合定位

基于 CARLA 仿真器的 LiDAR-IMU-GNSS 融合定位实验。

## 文件结构

```
src/lab_code/ch27_lab/
├── README.md               # 本文件
├── tf_broadcaster.py       # TF2 坐标树广播节点
├── ekf_localization.yaml   # robot_localization EKF 配置
└── localization_eval.py    # 定位精度评估节点
```

## 快速开始

### 1. 启动 CARLA 仿真

```bash
# 终端1: 启动 CARLA 服务器
./CarlaUE4.sh -quality-level=Low

# 终端2: 启动 ROS2 桥接
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
```

### 2. 启动 LiDAR SLAM

```bash
# 终端3: 启动 FAST-LIO2
ros2 launch fast_lio mapping.launch.py
```

### 3. 启动 EKF 融合

```bash
# 终端4: TF 广播
python3 src/lab_code/ch27_lab/tf_broadcaster.py

# 终端5: EKF 融合
ros2 run robot_localization ekf_node \
    --ros-args --params-file src/lab_code/ch27_lab/ekf_localization.yaml
```

### 4. 定位评估

```bash
# 终端6: 录制数据
ros2 bag record -o ch27_eval \
    /carla/ground_truth /odometry/filtered

# 终端7: 评估精度
python3 src/lab_code/ch27_lab/localization_eval.py \
    --bag ch27_eval/ch27_eval.db3
```

## 坐标系约定

| Frame | 描述 | 父坐标系 |
|-------|------|---------|
| map | 绝对世界坐标系 (UTM) | - |
| odom | 连续里程计坐标系 | map |
| base_link | 车辆本体 (后轴中心) | odom |
| lidar_link | LiDAR 安装位置 | base_link |
| imu_link | IMU 安装位置 | base_link |
| gps_link | GNSS 天线相位中心 | base_link |

## 依赖

- ROS 2 Jazzy
- robot_localization
- CARLA 0.9.16
- carla_ros_bridge
- EVO (pip install evo)
- FAST-LIO2 (或任意 LiDAR SLAM)

---

## 安装与编译

```bash
pip install numpy matplotlib
sudo apt install ros-jazzy-robot-localization   # EKF (ekf_localization.yaml)
```

## 运行方法

```bash
# TF 广播(需 ROS2):
python tf_broadcaster.py
# 使用 robot_localization 运行 EKF:
ros2 run robot_localization ekf_node --ros-args --params-file ekf_localization.yaml
# 离线评估轨迹(ATE/RPE):
python localization_eval.py --bag /path/to/rosbag2 --gt-topic /groundtruth --est-topic /odometry/filtered
```

## 运行结果

`localization_eval.py` 使用录制的 rosbag 计算 ATE/RPE 与时间对齐；完整验证需要 CARLA、ROS 2 Jazzy 和 rosbag 数据。本目录未提供独立离线测试套件。
