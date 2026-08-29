# topic_demo

第 3 章实验包：话题通信练习。

- 包类型：`ament_python`
- ROS 2 Jazzy

## 简介

本包用于练习 ROS 2 话题通信机制，涵盖发布订阅、QoS 配置以及通过 `/cmd_vel` 控制机器人运动。

## 节点 / 可执行说明

| 节点 | 功能 |
| --- | --- |
| `gps_pub` | 发布 `/gps_position`（`Point` 消息），x 递增、y = 2x + 1 |
| `gps_sub` | 订阅 `/gps_position`，计算点到原点的距离 |
| `qos_pub` | 发布双话题 `/qos_reliable` 与 `/qos_best_effort`，演示 QoS 差异 |
| `square_driver` | 发布 `/cmd_vel`，使机器人走出 1m × 1m 的正方形 |

## 构建命令

## 安装

```bash

source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src/lab_code/ch03_lab --ignore-src -r -y
```

## 构建命令

```bash
cd <workspace>
colcon build --symlink-install --packages-select topic_demo
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

## 运行步骤

```bash

ros2 run topic_demo gps_pub

ros2 run topic_demo gps_sub

ros2 run topic_demo qos_pub

# 正方形驾驶（需先启动 Gazebo 仿真）
ros2 run topic_demo square_driver
```

## 验证方法

```bash
ros2 topic echo /gps_position --once
ros2 topic echo /qos_reliable --once
colcon test --packages-select topic_demo
```

测试验证 GPS 点构造、距离计算和 `Twist` 命令逻辑，不等待 5 秒或启动图形界面。

## 运行结果截图

![topic_demo 运行结果](../docs/images/result.png)
