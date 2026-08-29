# 第 4 章：服务通信

本章使用 `service_demo` 演示 ROS 2 服务端和客户端，通过 `example_interfaces/srv/AddTwoInts` 完成两个整数相加。服务端回调可以单独测试，测试不会等待服务发现或启动图形界面。

## 安装

需要 ROS 2 Jazzy、`rclpy`、`example_interfaces` 和 Python pytest：

```bash

source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src/lab_code/ch04_lab --ignore-src -r -y
```

## 构建

```bash
colcon build --symlink-install --packages-select service_demo
source install/setup.bash
```

## 运行

终端 1 启动服务端：

```bash

ros2 run service_demo server
```

终端 2 启动客户端，默认调用 `5 + 3`，也可以传入两个整数：

```bash
ros2 run service_demo client
ros2 run service_demo client 12 -4
```

## 验证

```bash

ros2 service list

ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 4}"

colcon test --packages-select service_demo

colcon test-result --verbose
```

预期服务调用返回 `sum: 7`，节点测试验证正负整数和服务回调响应。

## 运行结果

实际运行结果截图：

![Ch04 服务通信运行结果](docs/images/result.png)
