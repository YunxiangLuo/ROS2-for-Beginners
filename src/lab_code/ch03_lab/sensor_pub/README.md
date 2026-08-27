# sensor_pub

第 3 章实验包：自定义消息发布。

- 包类型：`ament_python`
- ROS 2 Jazzy
- 依赖：`sensor_interfaces`

## 简介

本包演示自定义接口消息的发布。节点 `sensor_pub_node` 周期性发布 `SensorData` 自定义消息，字段包括温度、湿度、气压和设备 ID。

## 节点 / 可执行说明

| 节点 | 话题 | 消息类型 | 字段 |
| --- | --- | --- | --- |
| `sensor_pub_node` | `/sensor_data` | `sensor_interfaces/msg/SensorData` | `temperature=25.5`, `humidity=60.0`, `pressure=1013.25`, `device_id="sensor_01"` |

## 构建命令

## 安装

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/lab_code/ch03_lab --ignore-src -r -y
```

## 构建命令

```bash
cd <workspace>
colcon build --symlink-install --packages-select sensor_interfaces sensor_pub
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

## 运行步骤

```bash
# 启动发布节点
ros2 run sensor_pub sensor_pub_node

# 另开终端验证
ros2 topic echo /sensor_data --once
```

## 验证方法

```bash
ros2 interface show sensor_interfaces/msg/SensorData
ros2 topic echo /sensor_data --once
colcon test --packages-select sensor_pub
colcon test-result --verbose
```

预期消息包含 `25.5` 摄氏度、`60.0` 百分比、`1013.25` hPa 和 `sensor_01`。测试只调用发布回调。

## 运行结果截图

![sensor_pub 运行结果](../docs/images/result.png)
