# topic_demo_py

ROS 2 Jazzy 下的 Python 话题通信示例（ament_python）。演示发布者与订阅者之间通过自定义 `Gps` 消息进行通信。

## 节点说明

| 节点 | 说明 |
| --- | --- |
| `pytalker` | 发布 `gps_info` 话题，消息类型 `Gps`；发布前对坐标做缩放（`x *= 1.03`、`y *= 1.01`）。 |
| `pylistener` | 订阅 `gps_info`，根据收到的坐标计算距离。 |

依赖：`topic_demo_interfaces`（提供 `Gps` 消息）。

## 构建

```bash
colcon build --symlink-install --packages-select topic_demo_py
```

## 运行

运行前先 source 环境：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

在终端 1 启动发布者：

```bash
ros2 run topic_demo_py pytalker
```

在终端 2 启动订阅者：

```bash
ros2 run topic_demo_py pylistener
```

## 接口话题表

| 话题 | 方向 | 消息类型 | 说明 |
| --- | --- | --- | --- |
| `gps_info` | 发布（pytalker）/ 订阅（pylistener） | `topic_demo_interfaces/msg/Gps` | GPS 坐标信息 |

## 测试

1 项测试通过（冒烟测试）。
