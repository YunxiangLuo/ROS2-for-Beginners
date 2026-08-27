# param_demo

第 6 章实验包：参数系统练习。

- 包类型：`ament_python`
- ROS 2 Jazzy

## 简介

本包用于练习 ROS 2 参数系统，包括参数声明、参数回调以及通过 launch 文件加载参数配置和联合仿真导航。

## 节点 / 可执行说明

| 节点 | 功能 |
| --- | --- |
| `param_node` | 声明参数并注册参数变化回调 |
| `speed_ctrl` | 速度控制器，演示通过参数调节运动速度 |

## Launch 文件

| 文件 | 功能 |
| --- | --- |
| `demo.launch.py` | 启动 `demo_nodes_py` 的 talker + listener |
| `combined_sim_nav.launch.py` | 联合仿真导航 |

## 配置文件

- `config/params.yaml`：节点参数配置

## 构建命令

```bash
cd <workspace>
colcon build --symlink-install --packages-select param_demo
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

## 运行步骤

```bash
# 运行参数节点
ros2 run param_demo param_node

# 启动 talker/listener demo
ros2 launch param_demo demo.launch.py

# 联合仿真导航
ros2 launch param_demo combined_sim_nav.launch.py
```
