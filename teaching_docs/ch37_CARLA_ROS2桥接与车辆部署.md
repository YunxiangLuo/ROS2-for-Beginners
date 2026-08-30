# 第37章 CARLA-ROS2 桥接与车辆部署

> **课程**：ROS2 Python 编程  
> **章节**：第37章  
> **课时**：2 课时（90 分钟）  
> **教学方式**：讲授 + 演示  

---

## 学习目标

本章学习目标包括：理解CARLA-ROS2 Bridge的架构原理与通信机制，掌握Ego Vehicle的Blueprint选取与生成流程，学会在RViz2中可视化传感器话题与TF树，掌握车辆控制接口的使用与模式切换。

## 37.1 CARLA-ROS2 桥接架构

### 37.1.0 Bridge 安装与验证

#### 安装步骤

CARLA ROS 2 Bridge 需要从源码编译。本课程使用 Ubuntu 24.04、ROS 2 Jazzy、
CARLA 0.9.16，并由仓库根目录的 `setup_course.sh` 固定 bridge commit 和依赖版本：

```bash
# 1. 在课程仓库根目录运行安装器
cd /path/to/Technologies-of-ROS2-Programming-master
bash setup_course.sh --with-carla
source ~/.config/ros2-course/env.bash

# 如果 CARLA 服务端运行在 Windows 主机，改用：
# bash setup_course.sh --carla-bridge-only

# 2. 手动验证 bridge 工作空间（安装器已完成依赖安装和构建）
source /opt/ros/jazzy/setup.bash
source ~/carla_ws/install/setup.bash

# 3. 安装器生成的环境文件会设置 CARLA Python 路径和 Bridge 参数
#    包括 CARLA_HOST、CARLA_PORT、CARLA_MAP、CARLA_BRIDGE_TIMEOUT

# 4. 验证安装
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py --show-args
# 期望：显示可用参数列表，无报错
```

#### 安装验证清单

| 检查项 | 命令 | 预期结果 |
|--------|------|---------|
| Python路径 | `echo $PYTHONPATH \| grep carla` | 包含 `PythonAPI/carla` |
| Bridge包 | `ros2 pkg list \| grep carla` | `carla_ros_bridge` 等包名 |
| 编译状态 | `colcon build --packages-select carla_ros_bridge` | 无错误，SUCCESS |
| 启动参数 | `ros2 launch carla_ros_bridge carla_ros_bridge.launch.py --show-args` | 显示参数列表 |
| 话题映射 | `ros2 interface package carla_msgs` | 显示消息类型列表 |

#### 常见问题与排障

| 问题 | 原因 | 解决 |
|------|------|------|
| `No module named 'carla'` | 未使用课程 CARLA venv | `source ~/.config/ros2-course/env.bash`，再检查 `python -c "import carla"` |
| `carla_msgs` 未找到 | 依赖未正确构建 | 确保 `ros-bridge/carla_msgs` 已构建：`colcon build --packages-select carla_msgs` |
| `colcon build` 编译失败 | Python 或 Jazzy 依赖不匹配 | 确认使用 Ubuntu 24.04、Python 3.12 和课程安装器创建的环境 |
| `rosdep install` 找不到依赖 | rosdep 未初始化 | `sudo rosdep init && rosdep update` |
| Bridge启动后无话题 | 传感器未配置或ego vehicle未生成 | 先运行 `spawn_ego.py` 生成车辆 |
| `derived-object-msgs` 缺失 | bridge 依赖未完整安装 | 在 bridge 工作空间运行 `rosdep install --from-paths src --ignore-src -r -y` |

### 37.1.1 Bridge 工作原理

CARLA-ROS2 Bridge (`carla_ros_bridge`) 是连接CARLA仿真器与ROS2生态的中间件。它作为ROS2节点运行，通过CARLA Python API与仿真器通信，将仿真数据转换为ROS2话题，同时将ROS2控制指令下发到仿真器。

```
┌─────────────────────────────────────────────────────┐
│                   ROS2 生态                           │
│  ┌─────────┐  ┌──────────┐  ┌──────────────────┐    │
│  │ RViz2   │  │ rqt_graph│  │ 自定义控制节点    │    │
│  └────┬────┘  └────┬─────┘  └────────┬─────────┘    │
│       │            │                  │              │
│  ┌────▼────────────▼──────────────────▼──────────┐  │
│  │           carla_ros_bridge Node                │  │
│  │  ┌─────────────────────────────────────────┐   │  │
│  │  │  话题订阅器 │ 话题发布器 │ 服务服务器    │   │  │
│  │  └─────────────────────────────────────────┘   │  │
│  │  ┌─────────────────────────────────────────┐   │  │
│  │  │       CARLA Python API 客户端            │   │  │
│  │  └─────────────────────────────────────────┘   │  │
│  └──────────────────────┬──────────────────────────┘  │
└─────────────────────────┼────────────────────────────┘
                          │ TCP / UDP
┌─────────────────────────▼──────────────────────────┐
│                   CARLA 仿真器                        │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐   │
│  │ 世界     │  │ 车辆     │  │ 传感器           │   │
│  │ (World)  │  │ (Vehicle)│  │ (Camera/Lidar)  │   │
│  └──────────┘  └──────────┘  └──────────────────┘   │
└─────────────────────────────────────────────────────┘
```

### 37.1.2 话题映射表

CARLA-ROS2 Bridge 将CARLA中的传感器数据映射为以下ROS2话题：

| CARLA 组件 | ROS2 话题名称 | ROS2 消息类型 | 说明 |
|:----------:|:-------------:|:-------------:|------|
| GNSS | `/carla/ego_vehicle/gnss` | `nav_msgs/Odometry` | GPS定位数据 |
| IMU | `/carla/ego_vehicle/imu` | `sensor_msgs/Imu` | 惯性测量单元 |
| RGB Camera | `/carla/ego_vehicle/rgb_front/image` | `sensor_msgs/Image` | 前视彩色图像 |
| RGB Camera | `/carla/ego_vehicle/rgb_front/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |
| Depth Camera | `/carla/ego_vehicle/depth_front/image` | `sensor_msgs/Image` | 深度图像 |
| Lidar | `/carla/ego_vehicle/lidar` | `sensor_msgs/PointCloud2` | 激光雷达点云 |
| Radar | `/carla/ego_vehicle/radar_front` | `sensor_msgs/RadarDetectionArray` | 毫米波雷达 |
| 车辆状态 | `/carla/ego_vehicle/vehicle_status` | `carla_msgs/CarlaEgoVehicleStatus` | 速度、朝向等 |
| 车辆控制 | `/carla/ego_vehicle/vehicle_control_cmd` | `carla_msgs/CarlaEgoVehicleControl` | 油门/刹车/转向 |
| 里程计 | `/carla/ego_vehicle/odometry` | `nav_msgs/Odometry` | 车辆里程计 |
| 碰撞检测 | `/carla/ego_vehicle/collision` | `carla_msgs/CarlaCollisionEvent` | 碰撞事件 |
| 车道入侵 | `/carla/ego_vehicle/lane_invasion` | `carla_msgs/CarlaLaneInvasionEvent` | 车道偏离 |

### 37.1.3 同步模式 vs 异步模式

Bridge 支持两种运行模式，通过参数 `synchronous_mode` 控制：

| 特性 | 同步模式 (Synchronous) | 异步模式 (Asynchronous) |
|:----:|:----------------------:|:-----------------------:|
| 步进控制 | ROS2控制仿真步进 | CARLA自主推进仿真 |
| 时序一致性 | 严格保证 | 可能有延迟 |
| 传感器数据 | 所有传感器同一时刻触发 | 各传感器独立触发 |
| 适用场景 | 强化学习、高精度控制 | 可视化、数据采集 |
| 性能开销 | 较高 | 较低 |
| 配置方法 | `synchronous_mode:=True` | `synchronous_mode:=False` |

```bash
# 同步模式启动
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py \
  host:="$CARLA_HOST" port:="$CARLA_PORT" timeout:="$CARLA_BRIDGE_TIMEOUT" \
  town:="$CARLA_MAP" synchronous_mode:=True

# 异步模式启动
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py \
  host:="$CARLA_HOST" port:="$CARLA_PORT" timeout:="$CARLA_BRIDGE_TIMEOUT" \
  town:="$CARLA_MAP" synchronous_mode:=False
```

### 37.1.4 官方要点——官方桥接架构：双向映射的完整形态

> 本节内容综合翻译自 CARLA 官方文档（模拟器时间与 ROS Bridge 章节）与官方 carla-ros-bridge 代码库（README 与示例启动文件），另参考 The Construct 的「Self-Driving Cars with ROS 2 and CARLA」课程、ROS 2 官方 TF2 文档与 Robotics Back-End 的调试教程。原文均为英文，此处为中文编译，供课后巩固与进阶阅读。

carla-ros-bridge 官方仓库以「一个 bridge 节点 + 传感器节点树」的方式组织：`carla_ros_bridge` 进程持有与模拟器的单一连接，内部按 role_name 为每辆 ego 车辆生成对应的传感器话题（`/carla/<role_name>/rgb_front`、`/carla/<role_name>/lidar` 等），并把 `carla_msgs/CarlaEgoVehicleControl` 的控制指令反向写回模拟器——这正是本章 37.1.1 桥接工作原理与 37.1.2 话题映射表的官方实现。官方 README 强调两点工程纪律：每个 ego 车辆的 `role_name` 必须先于 spawn 在模拟器侧注册（否则 bridge 不为其创建话题），且传感器清单（`sensor_definition`）在启动文件里静态声明，新增传感器需要重启 bridge——对应 37.2.3 对 role_name 的讲解。

### 37.1.5 官方要点——同步模式与时间同步：仿真时钟的官方玩法

CARLA 官方文档在《Simulation time》与 ROS Bridge 章节给出两条铁律：其一，数据采集必须开同步模式（`synchronous_mode=True` + `sensor_tick`），由 `client.tick()` 统一推进仿真世界，否则多传感器时间戳错位、点云与图像对不上——本章 37.1.3 的同步/异步对比正是此节的中文编译；其二，ROS 侧必须 `use_sim_time:=true` 并让 bridge 把 CARLA 时钟发布为 `/clock`（bridge 默认行为），否则 TF 帧被 ROS 判定为过期，RViz 可视化会「跳帧」。官方还提示：同步模式下若某个传感器节点回复慢，bridge 的 `synchronous_mode_timeout` 会报超时，此时应调大该参数或减少传感器数量，而不是关闭同步。

## 37.2 Ego Vehicle 部署

### 37.2.1 Blueprint 选取

CARLA中的每种车辆都有唯一的Blueprint ID。常用Ego Vehicle Blueprint：

```python
import carla

# 连接CARLA客户端
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# 获取所有可用车辆Blueprint
blueprints = world.get_blueprint_library().filter('vehicle.*')
for bp in blueprints:
    print(f"{bp.id} | {bp.get_attribute('role_name').recommended_values}")

# 常用Ego Vehicle Blueprint
vehicle_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
# 或: vehicle.toyota.prius / vehicle.audi.etron / vehicle.lincoln.mkz2017
```

### 37.2.2 生成流程

Ego Vehicle 的完整生成流程：

```
1. 连接CARLA服务器
        │
2. 获取世界对象 (World)
        │
3. 选择Blueprint
        │
4. 设置角色名 (role_name = "ego_vehicle")
        │
5. 选择生成点 (Spawn Point)
        │
6. 生成车辆 (spawn_actor)
        │
7. 附加传感器 (Camera, Lidar, IMU...)
        │
8. 设置车辆灯光 (可选)
```

```python
# spawn_ego.py — 完整生成示例

def spawn_ego_vehicle(world, spawn_point_index=0):
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find('vehicle.tesla.model3')

    # 设置 role_name — Bridge 通过 role_name 识别Ego Vehicle
    vehicle_bp.set_attribute('role_name', 'ego_vehicle')
    vehicle_bp.set_attribute('color', '255,0,0')  # 红色

    # 获取生成点
    spawn_points = world.get_map().get_spawn_points()
    if spawn_point_index >= len(spawn_points):
        spawn_point_index = 0
    spawn_point = spawn_points[spawn_point_index]

    # 生成车辆
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    print(f"Ego Vehicle spawned at: {spawn_point.location}")
    return vehicle
```

### 37.2.3 role_name 的作用

`role_name` 是Bridge识别Ego Vehicle的关键属性。一个Bridge实例默认管理 `role_name="ego_vehicle"` 的车辆。多个Ego Vehicle可通过不同role_name区分：

| role_name | 对应Bridge话题前缀 | 用途 |
|:---------:|:------------------:|------|
| `ego_vehicle` | `/carla/ego_vehicle/*` | 主控车辆 |
| `hero` | `/carla/hero/*` | 第二控制车辆 |
| `vehicle_1` | `/carla/vehicle_1/*` | 其他受控车辆 |

### 37.2.4 Spawn Point 选取

```python
# 获取所有生成点并在RViz中可视化
spawn_points = world.get_map().get_spawn_points()
for i, sp in enumerate(spawn_points):
    print(f"[{i:3d}] x={sp.location.x:.2f}  y={sp.location.y:.2f}  "
          f"yaw={sp.rotation.yaw:.1f}")

# 选取特定位置的生成点
selected_sp = spawn_points[10]  # 索引10的位置
vehicle = world.spawn_actor(vehicle_bp, selected_sp)
```

## 37.3 RViz2 可视化

### 37.3.1 传感器话题可视化

在RViz2中添加以下显示以可视化CARLA传感器数据：

| 显示类型 | 话题 | 说明 |
|:--------:|:----:|------|
| Image | `/carla/ego_vehicle/rgb_front/image` | 前视RGB图像 |
| PointCloud2 | `/carla/ego_vehicle/lidar` | 激光雷达点云 |
| Odometry | `/carla/ego_vehicle/odometry` | 车辆里程计路径 |
| TF | `/tf` | 坐标变换树 |
| Axes | 固定坐标系 `map` | 坐标轴参考 |

```bash
# 启动RViz2并加载CARLA配置
rviz2 -d src/carla_ros_bridge/rviz/carla_bridge.rviz
```

### 37.3.2 TF 树结构

CARLA-ROS2 Bridge 发布的TF树：

```
map
 └── odom
      └── ego_vehicle
           ├── ego_vehicle/rgb_front
           ├── ego_vehicle/lidar
           ├── ego_vehicle/imu
           └── ego_vehicle/gnss
```

通过 `tf2_tools` 查看TF树：

```bash
ros2 run tf2_tools view_frames.py
evince frames.pdf  # 查看生成的TF树PDF
```

### 37.3.3 自定义Display配置

```xml
<!-- carla_rviz_config.rviz -->
<Tuple key="rviz" value="3">
  <Tuple key="Displays">
    <Tuple key="Grid" value="1">
      <Attribute key="Class" value="rviz_default_plugins/Grid"/>
      <Attribute key="Frame" value="map"/>
      <Attribute key="Plane Cell Count" value="25"/>
    </Tuple>
    <Tuple key="RobotModel" value="1">
      <Attribute key="Class" value="rviz_default_plugins/RobotModel"/>
      <Attribute key="Robot Description" value="robot_description"/>
    </Tuple>
    <Tuple key="Lidar" value="1">
      <Attribute key="Class" value="rviz_default_plugins/PointCloud2"/>
      <Attribute key="Topic" value="/carla/ego_vehicle/lidar"/>
      <Attribute key="Style" value="Points"/>
    </Tuple>
    <Tuple key="Camera" value="1">
      <Attribute key="Class" value="rviz_default_plugins/Image"/>
      <Attribute key="Topic" value="/carla/ego_vehicle/rgb_front/image"/>
    </Tuple>
  </Tuple>
</Tuple>
```

### 37.3.4 官方要点——Ego 车辆部署与 TF 树的官方约定

官方 carla-ros-bridge 把车牌信息与静态传感器（如相机）发布到 TF 树中，坐标为 `map → <role_name> → 各传感器 frame`，其命名规则与本章 37.3.2 的 TF 树结构一致：bridge 以 `role_name` 为 TF 前缀（`/ego_vehicle`），相机/激光雷达以其在 CARLA 中的挂载点命名。ROS 2 官方 TF2 文档与本仓库 37.3.1 的做法一致：先用 `ros2 run tf2_tools view_frames` 生成 TF 图核对父子关系，再谈传感器融合。官方示例 `spawn_object` 与手动 spawn 脚本的差异也值得注意：bridge 官方推荐用 launch 文件统一管理 vehicle + sensors 的生成，才能保证话题、TF 与控制接口同时就绪。

## 37.4 车辆控制接口

### 37.4.1 控制消息格式

车辆控制通过 `/carla/ego_vehicle/vehicle_control_cmd` 话题发送，消息类型为 `carla_msgs/CarlaEgoVehicleControl`：

```
# CarlaEgoVehicleControl.msg
float32 throttle       # 油门 [0.0, 1.0]
float32 brake          # 刹车 [0.0, 1.0]
float32 steer          # 转向 [-1.0, 1.0] (负=左, 正=右)
bool hand_brake        # 手刹
bool reverse           # 倒车
bool manual_gear_shift # 手动换挡
int32 gear             # 档位
```

### 37.4.2 手动控制模式

```python
import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl

class ManualControlNode(Node):
    def __init__(self):
        super().__init__('manual_control_node')
        self.pub = self.create_publisher(
            CarlaEgoVehicleControl,
            '/carla/ego_vehicle/vehicle_control_cmd',
            10)

    def send_control(self, throttle=0.0, brake=0.0, steer=0.0):
        cmd = CarlaEgoVehicleControl()
        cmd.throttle = throttle
        cmd.brake = brake
        cmd.steer = steer
        cmd.reverse = False
        cmd.hand_brake = False
        self.pub.publish(cmd)
        self.get_logger().info(
            f'Control: throttle={throttle:.2f}, steer={steer:.2f}')
```

### 37.4.3 自动/手动模式切换

通过服务调用实现模式切换：

```bash
# 启用自动驾驶模式
ros2 service call /carla/ego_vehicle/enable_autopilot \
  std_srvs/srv/SetBool "{data: true}"

# 禁用自动驾驶模式（切回手动）
ros2 service call /carla/ego_vehicle/enable_autopilot \
  std_srvs/srv/SetBool "{data: false}"
```

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class AutopilotSwitcher(Node):
    def __init__(self):
        super().__init__('autopilot_switcher')
        self.cli = self.create_client(SetBool,
            '/carla/ego_vehicle/enable_autopilot')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for autopilot service...')

    def set_autopilot(self, enabled: bool):
        req = SetBool.Request()
        req.data = enabled
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(
                f'Autopilot {"enabled" if enabled else "disabled"}')
        return future.result()

    def toggle(self):
        """手动/自动模式切换"""
        current = self.get_autopilot_state()
        self.set_autopilot(not current)
```

### 37.4.4 键盘控制示例

使用 `keyboard` 库实现键盘控制节点：

| 按键 | 控制指令 | 值 |
|:----:|:--------:|:--:|
| W | 油门增加 | +0.05 |
| S | 刹车增加 | +0.05 |
| A | 左转向 | -0.05 |
| D | 右转向 | +0.05 |
| Space | 手刹/急停 | brake=1.0 |
| R | 倒车切换 | reverse toggle |
| P | 自动驾驶切换 | autopilot toggle |

### 37.4.5 官方要点——从手动到自动驾驶：官方控制接口细读

carla_msgs 的 `CarlaEgoVehicleControl` 官方消息体与本章 37.4.1 完全对应：`throttle`（0~1）、`steer`（-1~1，右正）、`brake`（0~1）、`hand_brake` 与 `gear`。官方 bridge 示例默认使用 `set_autopilot(True)` 走 CARLA 内置规划器做基线驾驶，再切换为外部发布 `vehicle_control_cmd` 话题——学习路径与 37.4.3 的自动/手动切换一致；官方文档提醒：由 autopilot 切回手动控制的瞬间要先把 `throttle/brake` 平滑归零再接本地控制，否则车辆会以切换前的油门冲出去。The Construct 课程在本章对应的实战是「键盘控制 + RViz 观察 + 记录 bag」，其作业要求学生在桥接话题上先做 `ros2 topic info` 与 `hz` 验证，再写控制节点——与 37.4.4 键盘控制示例一致。

## 本章小结

CARLA-ROS2 Bridge 是连接仿真与ROS2的关键中间件，实现了传感器数据和车辆控制的完整双向映射。Ego Vehicle 通过Blueprint选取 → role_name设置 → Spawn Point确定 → spawn_actor 的流程生成。RViz2 可订阅Bridge发布的传感器话题实现数据可视化，TF树提供了完整的坐标关系。车辆控制接口支持手动和自动驾驶两种模式，可通过Service实现动态切换。

学习材料：
- carla-ros-bridge 官方代码库：https://github.com/carla-simulator/ros-bridge
- CARLA 官方文档 —— 模拟器时间与 ROS Bridge 章节：https://carla.readthedocs.io/
- The Construct —— Self-Driving Cars with ROS 2 and CARLA 课程：https://www.theconstructsim.com/
- ROS 2 官方文档 —— TF2 与 time 同步：https://docs.ros.org/
- Robotics Back-End —— 话题与 TF 调试教程：https://roboticsbackend.com/
