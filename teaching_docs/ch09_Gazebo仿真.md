# 第9章：Gazebo 仿真

> **课程**：ROS2 Python 编程  
> **章节**：第9章  
> **课时**：2 课时（90 分钟）  
> **教学方式**：讲授 + 演示  

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

### 知识点 9.1.3：官方要点——Gazebo 与 ROS 2 集成架构

官方 Gazebo Simulator 教程（Advanced/Simulators 分类）概述了 Gazebo 的两大版本与对应的 ROS 2 集成包：经典 Gazebo Classic（11.x，对应 `gazebo_ros_pkgs`）与新代 Gazebo（Ignition/Gz sim，对应 `ros_gz` 系列包）。本章使用的 `gazebo_ros` 属于前者，其架构为「Gazebo 物理引擎 + ROS 2 桥接层」：仿真世界运行在 Gazebo 进程中，ROS 2 侧通过桥接插件收发话题与服务。官方教程以 turtlesim 之外的差速小车为例，演示了 `ros_gz_bridge`/`gazebo_ros` 双向桥接的启动方式，与本章 9.1.2 节的集成原理一一对应。

The Construct 的课程强调版本配对的重要性：ROS 2 Humble 官方默认搭配 Gazebo Classic 11 与 Gz Garden，混用不同代际的插件会直接报「未找到插件库」错误，实验前应先核对 `package.xml` 中的仿真依赖。

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

### 知识点 9.2.2：官方要点——世界文件与机器人生成

Gazebo 的 SDF 世界文件定义了光照、地面、物理参数与模型列表，9.2 节启动文件中的 `empty.world`（仓库 `robot_sim_demo` 则为 `worlds/museum.sdf`）即采用该格式。官方教程推荐的工作流是：`ros2 launch gazebo_ros gazebo.launch.py world:=...` 指定世界文件启动仿真器，再用 `spawn_entity.py` 脚本把 URDF/XACRO 描述的机器人注入世界——脚本通过 `/gazebo/spawn_entity` 服务完成调用，参数 `-topic robot_description` 可直接复用 robot_state_publisher 发布的模型描述，避免文件路径硬编码。

注入后的坐标系规范值得注意：仿真世界的 `world` 坐标系与机器人 `base_footprint` 通过 `odom` 帧衔接，这正是第 7 章 TF 树在仿真中的落地。Articulated Robotics 的教程进一步演示了把「世界文件 + 模型 + Launch」打包为独立仿真包（如 `my_robot_gazebo`）的组织方式，与仓库中 `robot_sim_demo` 的包结构一致。

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

### 知识点 9.3.4：官方要点——传感器插件配置要点

本章 9.3 节使用的激光雷达与相机，依赖 Gazebo 侧的传感器插件把仿真数据流发布为 ROS 2 话题。经典插件的典型配置：差速底盘插件 `libgazebo_ros_diff_drive.so`（订阅 `/cmd_vel`、发布 `/odom` 与关节状态）、激光插件 `libgazebo_ros_ray_sensor.so`（发布 `sensor_msgs/LaserScan`）、相机插件 `libgazebo_ros_camera.so`（发布 `sensor_msgs/Image` 与相机信息）。插件在 URDF 的 `<gazebo>` 扩展标签中声明，输出话题名、更新频率与噪声模型均可参数化。

工程要点有三：其一，插件输出的话题帧名必须与 TF 树一致（如 `laser` 帧需在 URDF 中真实存在），否则 RViz 中 `/scan` 无法显示——本章练习 9.6 即可复现该现象；其二，传感器更新频率受 Gazebo 实时因子影响，CPU 负载过高时 `ros2 topic hz` 会明显低于设定值；其三，图像类话题带宽大，跨机调试建议配合 `compress_image` 或降分辨率。

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

### 知识点 9.4.3：官方要点——ros2_control 与差速驱动集成

官方 ros2_control 文档（control.ros.org）与 gz_ros2_control 项目把「控制器框架 + 仿真后端」的分工讲得最清楚：`Controller Manager` 加载 diff_drive_controller 等通用控制器，硬件接口由 `gz_ros2_control` 插件代理——它把 Gazebo 的关节状态注入 `JointStateBroadcaster`、把速度指令下发给仿真物理引擎，使同一套控制器配置在仿真与实机间无缝切换。本章 9.4 节的 `diff_drive_controller` 配置、YAML 参数文件与 Launch 集成正是该文档流程的中文落地。

配置时的三个常见坑与官方 FAQ 一致：`use_sim_time` 必须设为 true，否则控制器与仿真时钟失配；`robot_state_publisher` 未启动会导致 TF 缺失、里程计漂移图异常；控制器未先 `spawner` 加载就发送 `/cmd_vel` 会静默丢弃。建议读者按「先 odom 反馈、再 cmd_vel 闭环、最后 RViz 轨迹显示」的顺序逐项验证，把本章练习 9.5 的遥控实验与练习 9.6 的传感器观测串联成完整的仿真闭环。

---

## 9.5 本章小结

Gazebo Classic 通过 ros_gz_bridge 与 ROS2 通信，URDF 经自动桥接转换为 SDF 后由 spawn_entity.py 加载；LiDAR（`gazebo_ros_ray_sensor`）、Camera（`gazebo_ros_camera`）与 IMU 等传感器通过 `<gazebo>` 插件配置接入；ros2_control 的 `diff_drive_controller` 实现差速运动控制，所有插件参数统一通过 YAML 文件配置。

---

## 9.6 练习题

**练习 9.1**：编写 `sim_bringup.launch.py`，启动 Gazebo + 空世界 + spawn 机器人。

**练习 9.2**：在 URDF 中添加 LiDAR 的 `<gazebo>` 插件配置，发布 `/scan` 话题。

**练习 9.3**：在 URDF 中添加 Camera 插件，验证 `/camera/image_raw` 话题。

**练习 9.4**：编写 `diff_drive_controller` YAML 配置文件，配置 wheel_separation 和 wheel_radius。

**练习 9.5**：发布 `/cmd_vel` 指令控制机器人运动：`ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}"`

**练习 9.6**：在 RViz2 中订阅 `/scan` 和 `/camera/image_raw`，观察传感器数据。

---

## 仿真结合实例（当前仓库）：ISCAS Museum Gazebo 传感器与底盘控制

### 目标与知识点对应

本实例使用仓库当前的 `robot_sim_demo`，完整验证 Gazebo 世界加载、机器人 spawn、ROS-Gazebo Bridge、LiDAR/相机数据和 `/cmd_vel` 底盘控制，替代旧的 Gazebo Classic 示例命令。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 终端 1：启动 Gazebo、机器人、桥接和 RViz
ros2 launch robot_sim_demo gazebo2.launch.py \
  gui:=true rviz:=true drive:=false
```

```bash
# 终端 2：检查桥接数据
source install/setup.bash
ros2 topic echo /clock --once
ros2 topic info /scan
ros2 topic echo /camera/camera_info --once
ros2 topic echo /odom --once
```

```bash
# 终端 3：发送短时速度指令并观察里程计
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}"
```

### 观察结果

Gazebo 中加载 ISCAS Museum 场景并生成 Wheeltec，RViz 可显示 RobotModel、TF 和 LaserScan；`/clock`、`/scan`、`/odom` 由 `ros_gz_bridge` 转成 ROS 2 消息，发布 `/cmd_vel` 后可在 `/odom` 观察运动变化；关闭终端 3 的发布命令后，机器人不再接受新的速度指令。

### 源码与运行证据

Launch 文件位于 `src/robot_sim_demo/launch/gazebo2.launch.py`，桥接配置为 `src/robot_sim_demo/config/gazebo2_bridge.yaml`，世界文件为 `src/robot_sim_demo/worlds/museum.sdf`，RViz 配置为 `src/robot_sim_demo/rviz/museum.rviz`，终端运行证据见 `lab_manuals/images/runtime/ch09_gazebo_headless.png`。当前入口使用 Gazebo Sim Harmonic；文中 Gazebo Classic 的 API 仅作为历史概念对照，不能与本实例的启动命令混用。

![ch09 Gazebo headless 运行输出](../lab_manuals/images/runtime/ch09_gazebo_headless.gif)


![Campus PUCRS headless 运行输出](../lab_manuals/images/runtime/campus_pucrs_headless.gif)


---

> 参考来源：
> - ROS 2 Documentation (Humble) —— Gazebo Simulator 教程：https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo-Simulator.html
> - Gazebo 官方文档：https://gazebosim.org/docs
> - gazebo_ros_pkgs（Gazebo Classic 与 ROS 2 集成）：https://github.com/ros-simulation/gazebo_ros_pkgs
> - gz_ros2_control（Gazebo 与 ros2_control 集成）：https://github.com/ros-controls/gz_ros2_control
> - ros2_control 官方文档：https://control.ros.org/
> - The Construct —— ROS 2 Basics in 5 Days：https://www.theconstructsim.com/
> - Articulated Robotics —— Gazebo / Simulation 系列视频：https://www.youtube.com/@ArticulatedRobotics