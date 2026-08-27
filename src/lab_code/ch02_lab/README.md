# 第 2 章：节点与日志

本章使用 `hello_pkg` 练习 ROS 2 节点、定时器、日志等级和 `/odom` 订阅。`odom_monitor` 使用完整四元数计算 yaw，适用于一般姿态消息，不再把 `2*z` 当作通用航向角。

## 安装

需要 ROS 2 Jazzy、`rclpy`、`nav_msgs` 和 Python pytest：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/lab_code/ch02_lab --ignore-src -r -y
```

## 构建

```bash
colcon build --symlink-install --packages-select hello_pkg
source install/setup.bash
```

## 运行

```bash
ros2 run hello_pkg hello_node
ros2 run hello_pkg logger_node
ros2 run hello_pkg odom_monitor
```

`odom_monitor` 需要另行启动会发布 `/odom` 的机器人或仿真节点；前两个节点不需要 Gazebo。

## 验证

```bash
ros2 node list
ros2 topic echo /odom --once
colcon test --packages-select hello_pkg
colcon test-result --verbose
```

测试直接调用节点回调和四元数转换，不启动 GUI 或等待仿真。

## 运行结果

实际运行结果截图：

![Ch02 节点与日志运行结果](docs/images/result.png)
