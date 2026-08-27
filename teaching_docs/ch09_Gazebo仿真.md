# 第9章：Gazebo 仿真

> **课程**：ROS2 Python 编程  
> **章节**：第9章  
> **课时**：2 课时（90 分钟）  

---

## 9.1 Gazebo 仿真环境概述

### 知识点 9.1.1：Gazebo 版本选择

| 特性 | Gazebo Classic (11) | Gazebo Sim (Ignition) |
|------|---------------------|----------------------|
| ROS2 支持 | ros_gz_bridge | 原生 ros_gz |
| 发行版搭配 | Humble + Classic | Iron+ 推荐 Ignition |
| 稳定性 | 成熟稳定 | 积极开发中 |

本章以 Gazebo Classic + ROS2 Humble 为主进行讲解。

### 知识点 9.1.2：URDF → SDF 转换桥

```bash
# Gazebo 内部使用 SDF 格式，ROS 2 自动通过 URDF→SDF 桥接
# 启动时在 launch 文件中指定 spawn 参数即可
ros2 launch robot_sim_demo_ros2 sim_bringup.launch.py use_gazebo:=true
```

Gazebo 通过 `libgazebo_ros_init.so` 插件加载机器人模型。

---

## 9.2 Spawn 机器人模型

### 知识点 9.2.1：Gazebo 启动 Launch 文件

```python
# launch/sim_bringup.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('sim_demo')
    world_path = os.path.join(pkg_path, 'worlds', 'empty.world')

    use_gazebo = LaunchConfiguration('use_gazebo', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_gazebo', default_value='true',
                              description='是否启动 Gazebo'),

        # 启动 Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('gazebo_ros'),
                '/launch/gazebo.launch.py'
            ]),
            launch_arguments={'world': world_path}.items(),
        ),

        # Spawn 机器人模型
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'xbot',
                '-topic', 'robot_description',
                '-x', '0.0', '-y', '0.0', '-z', '0.1',
            ],
            output='screen',
        ),
    ])
```

程序 9-1：最小 Gazebo 启动文件，通过 spawn_entity 加载机器人。

---

## 9.3 传感器插件配置

### 知识点 9.3.1：LiDAR 传感器插件

```xml
<!-- 在 URDF/XACRO 中添加 Gazebo 插件 -->
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <pose>0 0 0 0 0 0</pose>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.12</min>
        <max>12.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/xbot</namespace>
        <argument>~/out:=scan</argument>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

程序 9-2：LiDAR 插件发布 `/scan` 话题。

### 知识点 9.3.2：Camera 传感器插件

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera_sensor">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image><width>640</width><height>480</height></image>
    </camera>
    <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
      <ros><namespace>/xbot</namespace></ros>
    </plugin>
  </sensor>
</gazebo>
```

### 知识点 9.3.3：IMU 传感器插件

```xml
<gazebo reference="imu_link">
  <sensor type="imu" name="imu_sensor">
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros><namespace>/xbot</namespace></ros>
    </plugin>
  </sensor>
</gazebo>
```

---

## 9.4 ros2_control 差速驱动

### 知识点 9.4.1：ros2_control 架构

```
硬件接口层：GazeboSystem (模拟编码器+电机)
    ↓
控制器管理层：diff_drive_controller (速度→左右轮速)
    ↓
ROS2 命令接口：/cmd_vel → 速度指令
```

### 知识点 9.4.2：Gazebo ros2_control 插件

```xml
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros2_control.so">
    <ros><namespace>/xbot</namespace></ros>
  </plugin>
</gazebo>
```

配套配置 YAML 文件定义控制器参数：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50

diff_drive_controller:
  type: diff_drive_controller/DiffDriveController
  ros__parameters:
    left_wheel_names:  ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.3
    wheel_radius: 0.05
    cmd_vel_timeout: 1.0
    publish_odom: true
```

程序 9-3：`diff_drive_controller` 接收 `/cmd_vel`，计算左右轮速写入硬件接口。

---

## 9.5 本章小结

1. Gazebo Classic 通过 ros_gz_bridge 与 ROS2 通信
2. URDF → SDF 自动桥接，通过 spawn_entity.py 加载
3. LiDAR (`gazebo_ros_ray_sensor`) / Camera (`gazebo_ros_camera`) / IMU 插件
4. ros2_control `diff_drive_controller` 实现差速运动控制
5. 所有插件通过 YAML 参数文件配置

---

## 9.6 练习题

**练习 9.1**：编写 `sim_bringup.launch.py`，启动 Gazebo + 空世界 + spawn 机器人。

**练习 9.2**：在 URDF 中添加 LiDAR 的 `<gazebo>` 插件配置，发布 `/scan` 话题。

**练习 9.3**：在 URDF 中添加 Camera 插件，验证 `/camera/image_raw` 话题。

**练习 9.4**：编写 `diff_drive_controller` YAML 配置文件，配置 wheel_separation 和 wheel_radius。

**练习 9.5**：发布 `/cmd_vel` 指令控制机器人运动：`ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}"`

**练习 9.6**：在 RViz2 中订阅 `/scan` 和 `/camera/image_raw`，观察传感器数据。