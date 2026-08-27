# service_demo_cpp

ROS 2 Jazzy 下的 C++ 服务通信示例（ament_cmake）。演示服务端与客户端之间通过自定义 `Greeting.srv` 进行请求/响应通信。

## 节点说明

| 节点 | 说明 |
| --- | --- |
| `server` | 提供 `/greetings` 服务，服务类型 `Greeting.srv`。 |
| `client` | 调用 `/greetings` 服务。 |

依赖：`service_demo_interfaces`（提供 `Greeting.srv`）。

## 构建

```bash
colcon build --symlink-install --packages-select service_demo_cpp
```

## 运行

运行前先 source 环境：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

在终端 1 启动服务端：

```bash
ros2 run service_demo_cpp server
```

在终端 2 启动客户端：

```bash
ros2 run service_demo_cpp client
```

## 接口话题表

| 接口 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/greetings` | `service_demo_interfaces/srv/Greeting` | 服务（server 提供 / client 调用） | 问候服务请求/响应 |
