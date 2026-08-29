# action_demo_cpp

ROS 2 Jazzy 下的 C++ 动作通信示例（ament_cmake）。演示动作服务端与客户端之间通过自定义 `DoDishes` action 进行目标/反馈/结果通信。

## 节点说明

| 节点 | 说明 |
| --- | --- |
| `dishes_server` | 提供 `/dishes` action，服务类型 `DoDishes`。 |
| `dishes_client` | 调用 `/dishes`，发送目标并接收反馈和结果。 |

依赖：`action_demo_interfaces`（提供 `DoDishes` action）、`rclcpp_action`。

## 构建

```bash

colcon build --symlink-install --packages-select action_demo_cpp
```

## 运行

运行前先 source 环境：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

在终端 1 启动动作服务端：

```bash

ros2 run action_demo_cpp dishes_server
```

在终端 2 启动动作客户端：

```bash
ros2 run action_demo_cpp dishes_client
```

## 接口话题表

| 接口 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/dishes` | `action_demo_interfaces/action/DoDishes` | 动作（dishes_server 提供 / dishes_client 调用） | 洗碗任务：目标、进度反馈、结果 |
