# 第 3 章：话题通信与自定义消息

本章包含 `topic_demo`、`sensor_interfaces` 和 `sensor_pub` 三个 ROS 包，覆盖发布订阅、QoS、速度话题和 `SensorData.msg` 自定义接口。

## 安装

需要 ROS 2 Jazzy、Python pytest，以及 `rosidl_default_generators`：

```bash

source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src/lab_code/ch03_lab --ignore-src -r -y
```

## 构建

```bash
colcon build --symlink-install --packages-select \
  sensor_interfaces sensor_pub topic_demo
source install/setup.bash
```

## 运行

```bash

ros2 run topic_demo gps_pub

ros2 run topic_demo gps_sub

ros2 run topic_demo qos_pub

ros2 run sensor_pub sensor_pub_node
```

需要控制仿真机器人时，再运行 `ros2 run topic_demo square_driver`；该节点会向 `/cmd_vel` 发布正方形轨迹速度。

## 验证

```bash
ros2 interface show sensor_interfaces/msg/SensorData
ros2 topic echo /gps_position --once
ros2 topic echo /sensor_data --once
colcon test --packages-select sensor_interfaces sensor_pub topic_demo
colcon test-result --verbose
```

测试直接检查消息字段、坐标距离、速度消息和自定义消息发布回调，不启动 Gazebo、RViz 或长时间轨迹。

## 运行结果

实际运行结果截图：

![Ch03 话题通信运行结果](docs/images/result.png)
