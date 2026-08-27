# sensor_interfaces

## 简介

本包是第 3 章实验（ch03_lab）的传感器接口定义包，基于 `ament_cmake` 构建。

它定义了实验中使用的自定义消息接口 `msg/SensorData.msg`，用于描述温湿度与气压等传感器采集数据。
本包不包含任何可执行节点，仅作为接口被 `sensor_pub`（传感器数据发布节点）依赖。

## 接口定义

### msg/SensorData.msg

描述一次传感器采集数据，包含温湿度、气压与传感器设备标识。

```
float64 temperature      # 温度 (℃)
float64 humidity         # 湿度 (%)
float64 pressure         # 气压 (hPa)
string device_id         # 传感器ID
```

## 构建命令

## 安装

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/lab_code/ch03_lab --ignore-src -r -y
```

## 构建命令

> 前提：ROS 2 Jazzy 已安装并完成环境配置。

```bash
colcon build --symlink-install --packages-select sensor_interfaces
```

## 验证命令

构建并 source 环境后，执行以下命令查看接口定义：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 interface show sensor_interfaces/msg/SensorData
colcon test --packages-select sensor_interfaces
colcon test-result --verbose
```

测试会实例化生成的 `SensorData` 消息并检查四个字段。

## 运行结果截图

![sensor_interfaces 运行结果](../docs/images/result.png)
