# 第9章 PPT：Gazebo 仿真

> 共 14 页

---

## P1 · 标题页
**Gazebo 仿真** | 第9章 | 2课时

## P2 · 学习目标
- 掌握 Gazebo 仿真环境搭建
- URDF → SDF 桥接与 spawn 机器人
- 配置传感器插件（LiDAR/Camera/IMU）
- ros2_control 差速驱动控制
- 话题发布控制机器人运动

## P3 · Gazebo 版本对比

| 特性 | Classic | Ignition |
|------|---------|----------|
| ROS2 桥接 | ros_gz_bridge | 原生集成 |
| 稳定性 | 成熟 | 积极开发 |
| 推荐 | Humble | Iron+ |

本章：Classic Gazebo + ROS2 Humble

## P4 · 仿真启动架构

```
sim_bringup.launch.py
├── gazebo.launch.py     ← 启动 Gazebo 服务器+客户端
├── spawn_entity.py      ← 加载机器人模型
├── robot_state_publisher ← 发布 TF
└── controller_manager   ← ros2_control
```

## P5 · Spawn 机器人

程序 9-1：
```python
Node(package='gazebo_ros', executable='spawn_entity.py',
     arguments=['-entity', 'xbot', '-topic', 'robot_description',
                '-x', '0.0', '-y', '0.0', '-z', '0.1'])
```

从 `/robot_description` 话题获取 URDF，动态生成 SDF

## P6 · 传感器架构图

```
         Gazebo 物理世界
               │
    ┌──────────┼──────────┐
    ▼          ▼          ▼
  LiDAR     Camera      IMU
 (ray)    (camera)     (imu)
    │          │          │
    ▼          ▼          ▼
  /scan   /image_raw  /imu
```

## P7 · LiDAR 插件配置

程序 9-2：
```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar">
    <ray>
      <scan><horizontal><samples>360</samples></horizontal></scan>
      <range><min>0.12</min><max>12.0</max></range>
    </ray>
    <plugin filename="libgazebo_ros_ray_sensor.so"/>
  </sensor>
</gazebo>
```

→ 发布 `sensor_msgs/LaserScan`

## P8 · Camera 插件

```xml
<gazebo reference="camera_link">
  <sensor type="camera">
    <camera><image><width>640</width><height>480</height></image></camera>
    <plugin filename="libgazebo_ros_camera.so"/>
  </sensor>
</gazebo>
```

→ 发布 `sensor_msgs/Image`

## P9 · IMU 插件

```xml
<gazebo reference="imu_link">
  <sensor type="imu">
    <plugin filename="libgazebo_ros_imu_sensor.so"/>
  </sensor>
</gazebo>
```

→ 发布 `sensor_msgs/Imu`

## P10 · ros2_control 架构

```
/cmd_vel (twist)
      ↓
controller_manager
      ↓
diff_drive_controller (速度分解)
      ↓
GazeboSystem (模拟硬件)
      ↓
/odom, /tf, /joint_states
```

## P11 · Controller 配置文件

程序 9-3：
```yaml
diff_drive_controller:
  type: diff_drive_controller/DiffDriveController
  ros__parameters:
    wheel_separation: 0.3
    wheel_radius: 0.05
    cmd_vel_timeout: 1.0
```

## P12 · 运动控制

```bash
# 前进 0.2 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" -1

# 原地旋转 0.5 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}" -1
```

## P13 · 本章要点

1. Gazebo 通过 pose/urdf→sdf 桥接 spawn 机器人
2. LiDAR/Camera/IMU 通过 `<sensor>`+`<plugin>` 配置
3. ros2_control diff_drive_controller 接收 cmd_vel 控制运动
4. 控制器参数通过 YAML 文件配置
5. RViz2 订阅仿真传感器话题可视化

## P14 · 课程总结
**ROS2 核心专题全部完成！** 后续：SLAM导航 / MoveIt2 机械臂