# action_demo_interfaces

## 简介

本包是 ROS 2 动作（Action）通信示例的接口定义包，基于 `ament_cmake` 构建。

它定义了动作示例中使用的自定义动作接口 `action/DoDishes.action`，用于在客户端与服务端之间执行长时任务（洗盘子）并周期性反馈进度。
本包不包含任何可执行节点，仅作为接口被 `action_demo_py`（Python 实现）与 `action_demo_cpp`（C++ 实现）依赖。

## 接口定义

### action/DoDishes.action

描述洗盘子任务：指定洗碗机编号，返回清洗总数，并周期性反馈完成百分比。

```
uint32 dishwasher_id             # 目标：洗碗机编号
---
uint32 total_dishes_cleaned      # 结果：清洗盘子总数
---
float32 percent_complete         # 反馈：完成百分比
```

## 构建命令

> 前提：ROS 2 Jazzy 已安装并完成环境配置。

```bash
colcon build --symlink-install --packages-select action_demo_interfaces
```

## 验证命令

构建并 source 环境后，执行以下命令查看接口定义：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 interface show action_demo_interfaces/action/DoDishes
```
