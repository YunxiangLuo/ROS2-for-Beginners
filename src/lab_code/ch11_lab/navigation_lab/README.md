# navigation_lab

第 11 章实验包：Nav2 导航实验。

- 包类型：`ament_python`
- ROS 2 Jazzy

## 简介

本包用于练习 Nav2 导航栈，涵盖目标点导航、航点跟随、故障恢复、扫描注入与导航监控，并通过 launch 文件启动 nav2_bringup 加载地图和参数。

## 节点 / 可执行说明

| 节点 | 功能 |
| --- | --- |
| `go_to_pose_demo` | 单点目标导航示例 |
| `follow_waypoints_demo` | 航点跟随导航示例 |
| `recovery_demo` | Nav2 故障恢复行为演示 |
| `scan_injector` | 注入模拟 `/scan` 数据 |
| `nav_monitor` | 导航状态监控 |
| `waypoint_patrol` | 航点巡逻 |

## Launch 文件

| 文件 | 功能 |
| --- | --- |
| `nav_bringup.launch.py` | 启动 `nav2_bringup`，加载地图与参数 |

## 配置文件

- `config/navigation_lab.yaml`

## 构建命令

```bash
cd <workspace>
colcon build --symlink-install --packages-select navigation_lab
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

## 运行步骤

```bash
# 启动 Nav2（需先启动仿真）
ros2 launch navigation_lab nav_bringup.launch.py

# 单点导航（需先启动仿真和 Nav2）
ros2 run navigation_lab go_to_pose_demo

# 航点跟随（需先启动仿真和 Nav2）
ros2 run navigation_lab follow_waypoints_demo
```
