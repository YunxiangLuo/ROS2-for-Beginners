# sim_demo

第 9 章 Gazebo 仿真实验包。

- 包类型：`ament_python`
- ROS 2 Jazzy + Gazebo Sim Harmonic

## 简介

本包通过 `sim_bringup.launch.py` 委托启动 `robot_sim_demo` 的 Gazebo Harmonic
入口，统一在 ISCAS Museum 场景中加载 Wheeltec 移动机器人。本包不再维护独立的
Gazebo Sim Harmonic 转发入口；移动机器人仿真以 `robot_sim_demo/gazebo2.launch.py` 为准。

## Launch 文件

| 文件 | 功能 |
| --- | --- |
| `sim_bringup.launch.py` | 委托 `robot_sim_demo/gazebo2.launch.py`，启动 Gazebo + Wheeltec + 桥接 |

### Launch 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `gui` | `true` | 启动 Gazebo GUI |
| `rviz` | `false` | 启动 RViz2 |
| `drive` | `true` | 启动自动巡航驱动 |

## 构建

```bash
cd <robot_sim_demo 工作区>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select sim_demo
source install/setup.bash
```

## 运行

```bash
ros2 launch sim_demo sim_bringup.launch.py
# 无 GUI、手动控制：
ros2 launch sim_demo sim_bringup.launch.py gui:=false drive:=false
```

启动后检查话题：

```bash
ros2 topic echo /clock --once
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

## 运行结果

启动 Gazebo 后，可见 ISCAS Museum 场景与 Wheeltec 机器人在中心位置自动巡航。

运行后可将截图保存至 `docs/images/gazebo_wheeltec.png`（首次运行前需创建该目录）：

```bash
mkdir -p docs/images
# 在 Gazebo GUI 中通过 Screenshot 插件或系统截屏工具保存
```
