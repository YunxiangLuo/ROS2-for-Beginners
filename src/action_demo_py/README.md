# action_demo_py

ROS 2 Jazzy 下的 Python 动作通信示例（ament_python）。演示动作服务端与客户端之间通过自定义 `DoDishes` action 进行目标/反馈/结果通信。

## 节点说明

| 节点 | 说明 |
| --- | --- |
| `dishes_server` | 提供 `/dishes` action。共 5 步，每步 0.5s；反馈进度从 20% 递增到 100%；结果 `total_dishes_cleaned = dishwasher_id * 5`。 |
| `dishes_client` | 发送目标 `dishwasher_id = 2`，接收反馈和结果。 |

依赖：`action_demo_interfaces`（提供 `DoDishes` action）。

## 构建

```bash
colcon build --symlink-install --packages-select action_demo_py
```

## 运行

运行前先 source 环境：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

在终端 1 启动动作服务端：

```bash
ros2 run action_demo_py dishes_server
```

在终端 2 启动动作客户端：

```bash
ros2 run action_demo_py dishes_client
```

也可使用命令行直接发送目标：

```bash
ros2 action send_goal /dishes action_demo_interfaces/action/DoDishes "{dishwasher_id: 2}"
```

## 接口话题表

| 接口 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/dishes` | `action_demo_interfaces/action/DoDishes` | 动作（dishes_server 提供 / dishes_client 调用） | 洗碗任务：目标、进度反馈、结果 |

## 测试

1 项测试通过（冒烟测试）。
