# slam_lab

第 10 章实验包：SLAM 实验。

- 包类型：`ament_python`
- ROS 2 Jazzy

## 简介

本包用于练习 SLAM 建图与定位，支持 slam_toolbox 在线建图、Cartographer 建图以及 AMCL 定位，并提供地图统计监控、初始位姿设置、AMCL 评估和 Cartographer 状态保存等工具节点。

## 节点 / 可执行说明

| 节点 | 功能 |
| --- | --- |
| `slam_monitor` | 地图统计信息监控 |
| `set_initial_pose` | 设置初始位姿 |
| `amcl_evaluator` | AMCL 定位精度评估 |
| `save_cartographer_state` | 保存 Cartographer 状态 |

## Launch 文件

| 文件 | 功能 |
| --- | --- |
| `online_mapping.launch.py` | slam_toolbox 在线建图 + 可选监控节点 |
| `cartographer_mapping.launch.py` | Cartographer 建图 |
| `amcl_localization.launch.py` | AMCL 定位 |

## 配置文件

- `config/nav2_localization.yaml`
- `config/mapper_params_online_async.yaml`
- `config/cartographer/*.lua`

## 构建命令

```bash
cd <workspace>
colcon build --symlink-install --packages-select slam_lab
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

## 运行步骤

```bash
# 在线建图（需先启动仿真并保证 /scan 话题存在）
ros2 launch slam_lab online_mapping.launch.py

# Cartographer 建图
ros2 launch slam_lab cartographer_mapping.launch.py

# AMCL 定位
ros2 launch slam_lab amcl_localization.launch.py
```
