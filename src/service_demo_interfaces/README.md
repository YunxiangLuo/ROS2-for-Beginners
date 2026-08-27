# service_demo_interfaces

## 简介

本包是 ROS 2 服务（Service）通信示例的接口定义包，基于 `ament_cmake` 构建。

它定义了服务示例中使用的自定义服务接口 `srv/Greeting.srv`，用于在客户端与服务端之间传递问候请求与反馈。
本包不包含任何可执行节点，仅作为接口被 `service_demo_py`（Python 实现）与 `service_demo_cpp`（C++ 实现）依赖。

## 接口定义

### srv/Greeting.srv

描述一次问候交互：客户端发送姓名与年龄，服务端返回问候反馈。

```
string name       # 请求：姓名
int32 age         # 请求：年龄
---
string feedback   # 响应：问候反馈
```

## 构建命令

> 前提：ROS 2 Jazzy 已安装并完成环境配置。

```bash
colcon build --symlink-install --packages-select service_demo_interfaces
```

## 验证命令

构建并 source 环境后，执行以下命令查看接口定义：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 interface show service_demo_interfaces/srv/Greeting
```
