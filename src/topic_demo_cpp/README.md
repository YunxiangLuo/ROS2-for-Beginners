# topic_demo_cpp

ROS 2 Jazzy 下的 C++ 话题通信示例（ament_cmake）。演示发布者与订阅者之间通过自定义 `Gps` 消息及 `std_msgs` 字符串消息进行通信。

## 节点说明

| 节点 | 说明 |
| --- | --- |
| `talker` | 发布 `gps_info` 话题，消息类型 `Gps`。 |
| `listener` | 同时订阅 `gps_info`（`Gps`）与 `/chatter`（`std_msgs/String`）。 |

依赖：`topic_demo_interfaces`（提供 `Gps` 消息）、`std_msgs`。

## 构建

```bash

colcon build --symlink-install --packages-select topic_demo_cpp
```

## 运行

运行前先 source 环境：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

在终端 1 启动发布者：

```bash

ros2 run topic_demo_cpp talker
```

在终端 2 启动订阅者：

```bash
ros2 run topic_demo_cpp listener
```

## 接口话题表

| 话题 | 方向 | 消息类型 | 说明 |
| --- | --- | --- | --- |
| `gps_info` | 发布（talker）/ 订阅（listener） | `topic_demo_interfaces/msg/Gps` | GPS 坐标信息 |
| `/chatter` | 订阅（listener） | `std_msgs/msg/String` | 字符串消息 |
