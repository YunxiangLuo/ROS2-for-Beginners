# service_demo_py

ROS 2 Jazzy 下的 Python 服务通信示例（ament_python）。演示服务端与客户端之间通过自定义 `Greeting.srv` 进行请求/响应通信。

## 节点说明

| 节点 | 说明 |
| --- | --- |
| `server_demo` | 提供 `/greetings` 服务，服务类型 `Greeting.srv`。 |
| `client_demo` | 调用 `/greetings`，发送请求 `name="HAN"`、`age=20`。 |

依赖：`service_demo_interfaces`（提供 `Greeting.srv`）。

## 构建

```bash
colcon build --symlink-install --packages-select service_demo_py
```

## 运行

运行前先 source 环境：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

在终端 1 启动服务端：

```bash
ros2 run service_demo_py server_demo
```

在终端 2 启动客户端：

```bash
ros2 run service_demo_py client_demo
```

## 接口话题表

| 接口 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/greetings` | `service_demo_interfaces/srv/Greeting` | 服务（server_demo 提供 / client_demo 调用） | 问候服务请求/响应 |

## 测试

1 项测试通过（冒烟测试）。
