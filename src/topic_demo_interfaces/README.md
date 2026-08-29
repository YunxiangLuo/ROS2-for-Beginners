# topic_demo_interfaces

## 简介

本包是 ROS 2 话题（Topic）通信示例的接口定义包，基于 `ament_cmake` 构建。

它定义了话题示例中使用的自定义消息接口 `msg/Gps.msg`，用于在发布者与订阅者之间传递 GPS 位置信息。

本包不包含任何可执行节点，仅作为接口被 `topic_demo_py`（Python 实现）与 `topic_demo_cpp`（C++ 实现）依赖。

## 接口定义

### msg/Gps.msg

描述 GPS 位置信息，包含状态字符串与平面坐标。

```
string state     # 状态标识
float32 x        # x 坐标
float32 y        # y 坐标
```

## 构建命令

> 前提：ROS 2 Jazzy 已安装并完成环境配置。

```bash

colcon build --symlink-install --packages-select topic_demo_interfaces
```

## 验证命令

构建并 source 环境后，执行以下命令查看接口定义：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 interface show topic_demo_interfaces/msg/Gps
```
