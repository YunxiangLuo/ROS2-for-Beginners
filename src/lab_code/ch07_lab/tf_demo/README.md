# tf_demo

第 7 章实验包：TF2 坐标变换练习。

- 包类型：`ament_python`
- ROS 2 Jazzy
- 依赖：`tf2_ros`、`tf2_geometry_msgs`、`geometry_msgs`

## 简介

本包用于练习 TF2 坐标变换系统，包括坐标系的广播与查询监听。

## 节点 / 可执行说明

| 节点 | 功能 |
| --- | --- |
| `tf_broadcaster` | 广播坐标变换 |
| `tf_listener` | 监听并查询坐标变换 |

## 构建命令

```bash

cd <workspace>

colcon build --symlink-install --packages-select tf_demo

source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

## 运行步骤

```bash
# 终端 1：广播坐标变换
ros2 run tf_demo tf_broadcaster

# 终端 2：监听坐标变换
ros2 run tf_demo tf_listener
```
