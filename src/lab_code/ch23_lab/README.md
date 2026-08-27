# 第23章 实验代码：CARLA-ROS2 桥接与车辆部署

本章实验围绕 CARLA 仿真器与 ROS2 的桥接通信，实现 Ego Vehicle 生成、传感器数据可视化和车辆控制。

## 文件说明

| 文件 | 用途 | 运行方式 |
|:----:|------|:--------:|
| `bridge_launch.sh` | 一键启动脚本：启动CARLA + ROS2 Bridge + Ego Vehicle | `bash bridge_launch.sh` |
| `spawn_ego.py` | Python脚本，通过CARLA API生成Ego Vehicle并设置传感器 | `python3 spawn_ego.py [options]` |
| `check_topics.py` | 话题验证脚本，列出并检查所有CARLA ROS2话题是否正常 | `python3 check_topics.py` |

## 快速启动

### 前置条件
- ROS 2 Jazzy 已安装并 sourced
- CARLA 0.9.16 已安装
- carla_ros_bridge 已编译到工作空间

### 完整运行流程

```bash
# 终端1：启动CARLA仿真器
./CarlaUE4.sh -quality-level=Low

# 终端2：一键启动Bridge + Ego Vehicle
cd src/lab_code/ch23_lab/
bash bridge_launch.sh

# 终端3：验证话题
python3 check_topics.py

# 终端4：RViz2可视化
rviz2

# 终端5：车辆控制（发布指令）
ros2 topic pub /carla/ego_vehicle/vehicle_control_cmd \
  carla_msgs/msg/CarlaEgoVehicleControl \
  "{throttle: 0.3, steer: 0.0, brake: 0.0}" --rate 10
```

### 分步启动

```bash
# 步骤1：启动CARLA（保持运行）
CarlaUE4.exe -quality-level=Low &

# 等待10秒让CARLA完全启动
sleep 10

# 步骤2：启动ROS2 Bridge
source ~/carla_ws/install/setup.bash
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py \
  synchronous_mode:=False &

# 等待5秒
sleep 5

# 步骤3：生成Ego Vehicle
python3 spawn_ego.py --spawn-point 15

# 步骤4：检查话题
python3 check_topics.py
```

## 常见问题

**Q: Bridge启动后没有 `/carla/ego_vehicle/*` 话题？**
A: 需要先生成Ego Vehicle并设置 `role_name=ego_vehicle`，Bridge才会创建对应话题。

**Q: spawn_ego.py报连接错误？**
A: 确保CARLA仿真器已启动，且端口（默认2000）未被占用。

**Q: RViz2中看不到点云？**
A: 检查 PointCloud2 显示的话题是否为 `/carla/ego_vehicle/lidar`，并确认Lidar传感器已附加到车辆。

---

## 安装与编译

```bash
# ROS 2 Jazzy + CARLA + ros-bridge（carla_ros_bridge）
pip install numpy
sudo apt install ros-jazzy-ros-bridge-carla-msgs-bridge  # 或源码编译 ros-bridge
```

## 运行方法

```bash
# 1. 启动 CARLA 服务器
# 2. 启动 ros-bridge:
bash bridge_launch.sh
# 3. 生成本车(带传感器):
python spawn_ego.py
# 4. 检查 ROS 话题与类型:
python check_topics.py --verbose
```

## 运行结果

在 CARLA、`carla_ros_bridge` 和 Ego Vehicle 均已启动后，`check_topics.py` 可用于检查桥接话题。本目录未提供独立离线测试套件。
