# msgs_demo_interfaces

## 简介

本包是从 ROS 1 `msgs_demo` 迁移而来的 ROS 2 接口定义包，基于 `ament_cmake` 构建。

它集中定义了一批用于机器人仿真的通用消息、服务与动作接口，涵盖运动学、传感、导航等常见数据结构，
便于在多个示例与实验包之间复用。本包不包含任何可执行节点。

接口依赖外部消息包：`geometry_msgs`、`nav_msgs`、`sensor_msgs`、`std_msgs`。

## 接口定义

### msg（消息）

| 名称 | 说明 |
| --- | --- |
| Accel | 加速度（线速度 + 角速度） |
| Echos | 超声回声数据 |
| Imu | IMU 惯性测量数据 |
| LaserScan | 激光雷达扫描数据 |
| Odometry | 里程计位姿与速度 |
| Point | 三维点 |
| Pose | 位姿（位置 + 朝向） |
| PoseStamped | 带时间戳的位姿 |
| PoseWithCovariance | 带协方差的位姿 |
| Power | 电源状态 |
| Quaternion | 四元数 |
| Twist | 速度（线速度 + 角速度） |
| TwistWithCovariance | 带协方差的速度 |
| Vector3 | 三维向量 |

### srv（服务）

| 名称 | 说明 |
| --- | --- |
| AddTwoInts | 两整数求和 |
| Empty | 空请求/响应 |
| GetMap | 获取地图 |
| GetPlan | 获取路径规划 |
| SetBool | 设置布尔量 |
| SetCameraInfo | 设置相机参数 |
| SetMap | 设置地图 |
| TalkerListener | 话题示例服务 |
| Trigger | 触发服务 |

### action（动作）

| 名称 | 说明 |
| --- | --- |
| AddTwoInts | 两整数求和（带反馈） |
| AutoDocking | 自动回充对接 |
| GetMap | 获取地图（带反馈） |
| MoveBase | 移动到目标点（带反馈） |

## 构建命令

> 前提：ROS 2 Jazzy 已安装并完成环境配置。

```bash
colcon build --symlink-install --packages-select msgs_demo_interfaces
```

## 验证命令

构建并 source 环境后，执行以下命令列出本包提供的全部接口：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 interface list | grep msgs_demo_interfaces
```
