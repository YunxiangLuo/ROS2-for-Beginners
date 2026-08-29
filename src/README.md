# ROS 2 仿真与教学

本工作区包含 50 个可由 colcon 发现和构建的 ROS 2 包，涵盖话题通信、服务通信、动作通信、参数系统、TF 坐标变换、URDF 建模、Gazebo 仿真、SLAM 建图、Nav2 自主导航、xArm6 机械臂仿真和 CARLA 自动驾驶仿真（接口/感知/规划/控制/安全/传感器），以及一个完整的 ISCAS Museum 仿真场景。`robot_sim_demo/wheeltec_robot_urdf/` 另含一个嵌套资源包 `package.xml`，不会被 colcon 作为独立包发现。

## 环境

| 组件 | 版本 |
|------|------|
| 操作系统 | Ubuntu 24.04 (WSL2) |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Sim Harmonic (v8) |
| 构建工具 | colcon + ament |

## 目录结构

```text

├── src/

│   ├── robot_sim_demo/              Gazebo 仿真核心包

│   │   ├── launch/gazebo2.launch.py 主启动入口（museum.sdf）

│   │   ├── launch/campus_pucrs.launch.py Campus PUCRS 启动入口

│   │   ├── models/                  机器人、博物馆、地面模型

│   │   ├── worlds/museum.sdf       仿真世界

│   │   ├── config/                  ROS-Gazebo 桥配置

│   │   ├── gui/                     Gazebo GUI 配置

│   │   └── wheeltec_robot_urdf/     Wheeltec URDF/STL 资源

│   ├── xarm/                        xArm6 + Gazebo Harmonic + MoveIt 2 仿真

│   ├── navigation_sim_demo_ros2/    Nav2 导航仿真

│   ├── slam_sim_demo_ros2/          SLAM 建图仿真

│   ├── tf_follower_ros2/            TF 跟随控制器

│   ├── topic_demo_py/               话题通信 (Python)

│   ├── topic_demo_cpp/              话题通信 (C++)

│   ├── topic_demo_interfaces/       话题消息接口

│   ├── service_demo_py/             服务通信 (Python)

│   ├── service_demo_cpp/            服务通信 (C++)

│   ├── service_demo_interfaces/     服务接口

│   ├── action_demo_py/              动作通信 (Python)

│   ├── action_demo_cpp/             动作通信 (C++)

│   ├── action_demo_interfaces/      动作接口

│   ├── param_demo_py/               参数系统 (Python)

│   ├── param_demo_cpp/              参数系统 (C++)

│   ├── tf_demo_py/                  TF 坐标变换 (Python)

│   ├── tf_demo_cpp/                 TF 坐标变换 (C++)

│   ├── name_demo_cpp/               命名空间与参数 (C++)

│   ├── msgs_demo_interfaces/       综合消息接口

│   ├── urdf_demo_ros2/            URDF/Xacro 建模演示

│   ├── av_carla_interfaces/         CARLA 自动驾驶共享接口（msg/srv/action）

│   ├── av_sensor_kit/               CARLA 传感器套件（配置/健康管理）

│   ├── av_perception_py/            感知（YOLO/DBSCAN/融合）

│   ├── av_planning_py/              规划（A*/航点/Navigate 动作）

│   ├── av_control_cpp/              车辆控制（PID/Pure Pursuit）

│   ├── av_safety_monitor/           安全监控（TTC/AEB/故障注入）

│   ├── course_lab_interfaces/      课程实验共享接口

│   ├── course_lab_utils/           课程实验共享实现

│   └── lab_code/                    教学实验包（Ch01-Ch31）

│       ├── ch01_lab/lifecycle_demo/  生命周期节点

│       ├── ch02_lab/hello_pkg/      节点与日志

│       ├── ch03_lab/topic_demo/      话题通信

│       ├── ch03_lab/sensor_pub/     自定义消息

│       ├── ch03_lab/sensor_interfaces/ 传感器接口

│       ├── ch04_lab/service_demo/   服务通信

│       ├── ch05_lab/action_demo/    动作通信

│       ├── ch06_lab/param_demo/     参数系统

│       ├── ch07_lab/tf_demo/        TF 坐标变换

│       ├── ch08_lab/urdf_demo/      URDF 建模

│       ├── ch09_lab/sim_demo/       Gazebo 仿真

│       ├── ch10_lab/slam_lab/       SLAM 建图

│       ├── ch11_lab/navigation_lab/  Nav2 导航

│       ├── ch12_lab/realsense_lab/  RealSense 相机

│       ├── ch13_lab/slam_bringup_lab/ SLAM 一键建图

│       ├── ch14_lab/nav_bringup_lab/  Nav2 一键导航

│       ├── ch15_lab/arm_joint_pub_lab/ xArm 关节发布

│       ├── ch17_lab/moveit_fk_ik_lab/  MoveIt FK/IK

│       ├── ch18_lab/moveit_pick_place_lab/ MoveIt 抓取放置

│       ├── ch19_lab/vision_detection_lab/ 视觉检测

│       ├── ch21_lab/vision_pickup_lab/   视觉引导抓取

│       └── ch22_lab/ ... ch31_lab/       CARLA 自动驾驶实验（脚本）

└── README.md                        本文件
```

## 安装

### 1. 安装 ROS 2 Jazzy

参考 [ROS 2 官方安装指南](https://docs.ros.org/en/jazzy/Installation.html)：

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander \
   ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server \
   ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui \
   ros-jazzy-rviz2 ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
   ros-jazzy-gz-ros2-control ros-jazzy-moveit \
   ros-jazzy-trac-ik-kinematics-plugin \
   python3-colcon-common-extensions
```

### 2. 准备 xArm 描述底层

`xarm_ros2_arm_only` 的运行依赖自定义 XBot Arm `xarm_description` `2.0.0`。该描述包不随本工作区提供，必须先在独立底层工作区中构建，并在构建或启动 xArm 前 source：

```bash

source /opt/ros/jazzy/setup.bash

cd /path/to/xarm_description_workspace

colcon build --symlink-install --packages-select xarm_description

source install/setup.bash
```

该描述包必须提供 `xarm_description/urdf/arm.urdf.xacro`，并采用 `arm_1_joint` 至 `arm_6_joint` 的关节命名。不要直接替换为使用 `joint1` 至 `joint6` 的 UFACTORY 官方描述包，除非同步迁移 Xacro、SRDF、控制器和 MoveIt 配置。

### 3. 克隆工作区

```bash
git clone https://github.com/YunxiangLuo/ROS2.git
cd ROS2
```

### 4. 编译全部包

```bash

source /opt/ros/jazzy/setup.bash

source /path/to/xarm_description_workspace/install/setup.bash

colcon build --symlink-install

source install/setup.bash
```

源码树当前由 `setup_course.sh` 发现 50 个可构建包；其中包含核心仿真、`course_lab_*`、实验包和 CARLA 自动驾驶系列。嵌套目录 `robot_sim_demo/wheeltec_robot_urdf/` 作为 `robot_sim_demo` 的数据资源安装，不单独计入构建包。

CARLA 系列额外依赖：

```bash
# CARLA 仿真器（ch22-ch31 实验需要，见 src/lab_code/ch22_lab/install_carla.sh）
# CARLA-ROS2 桥接与 Python API
pip install carla numpy opencv-python
# 可选：YOLO 检测
pip install ultralytics
```

## 包清单

### 仿真核心

| 包名 | 类型 | 说明 |
|------|------|------|
| `robot_sim_demo` | Python | ISCAS Museum / Campus PUCRS Gazebo 仿真：Wheeltec 机器人、传感器桥、巡航驱动 |
| `xarm_ros2_arm_only` | Python | xArm6 纯机械臂仿真：Gazebo Harmonic、ros2_control、MoveIt 2 和 RViz |
| `navigation_sim_demo_ros2` | Python | Nav2 导航栈：地图、AMCL、规划、控制 |
| `slam_sim_demo_ros2` | Python | slam_toolbox 在线建图 |
| `tf_follower_ros2` | Python | TF 跟随控制器：基于坐标变换的速度控制 |

### 通信示例

| 包名 | 语言 | 说明 |
|------|------|------|
| `topic_demo_py` | Python | 话题通信（GPS 数据发布/订阅） |
| `topic_demo_cpp` | C++ | 话题通信（GPS 数据发布/订阅） |
| `service_demo_py` | Python | 服务通信（Greeting 请求/响应） |
| `service_demo_cpp` | C++ | 服务通信（Greeting 请求/响应） |
| `action_demo_py` | Python | 动作通信（DoDishes 洗碗任务） |
| `action_demo_cpp` | C++ | 动作通信（DoDishes 洗碗任务） |

### 接口包

| 包名 | 说明 |
|------|------|
| `topic_demo_interfaces` | `msg/Gps`（state, x, y） |
| `service_demo_interfaces` | `srv/Greeting`（name, age → feedback） |
| `action_demo_interfaces` | `action/DoDishes`（dishwasher_id → total_dishes_cleaned, percent_complete） |
| `msgs_demo_interfaces` | ROS 1 迁移的综合消息接口（24 个 msg/srv/action） |
| `sensor_interfaces` | `msg/SensorData`（temperature, humidity, pressure, device_id） |

### 教学支持包

| 包名 | 类型 | 说明 |
|------|------|------|
| `course_lab_interfaces` | C++ | 课程实验共享接口：`srv/ValidateRecipe`、`srv/ReadLabel`、`action/Pipeline`、`msg/MarkerPose` |
| `course_lab_utils` | Python | 课程实验共享实现：MoveIt 演示、视觉检测、AR 抓取、机械臂控制（lab_code ch17-ch21 依赖） |

### CARLA 自动驾驶（av 系列）

| 包名 | 类型 | 说明 |
|------|------|------|
| `av_carla_interfaces` | C++ | 自动驾驶共享接口：`ControlCmd`/`EgoState`/`Waypoint`/`PerceptionObject` 等 msg、`PlanPath` srv、`Navigate` action |
| `av_sensor_kit` | Python | CARLA 传感器套件：配置预设（YAML 读写）与传感器健康管理节点 |
| `av_perception_py` | Python | 感知节点：YOLO 目标检测（HSV 兜底）、LiDAR DBSCAN 聚类、相机-LiDAR 前融合 |
| `av_planning_py` | Python | 规划节点：A* 栅格全局规划、航点生成、`/navigate` 动作服务 |
| `av_control_cpp` | C++ | 车辆控制器：纵向 PID、横向 Pure Pursuit、一体化车辆控制 |
| `av_safety_monitor` | Python | 安全监控：TTC 三级预警、AEB 紧急制动、碰撞事件处理、故障注入工具 |

### 其他示例

| 包名 | 语言 | 说明 |
|------|------|------|
| `param_demo_py` | Python | 参数声明、修改、删除、回调 |
| `param_demo_cpp` | C++ | 参数系统 |
| `tf_demo_py` | Python | TF2 广播、监听、坐标变换 |
| `tf_demo_cpp` | C++ | TF2 广播、监听、四元数转换 |
| `name_demo_cpp` | C++ | 命名空间、节点名、参数 |
| `urdf_demo_ros2` | Python | URDF/Xacro 建模演示：mybot 模型、四种 RViz 显示 launch |

### 教学实验包（lab_code，31 章）

**ROS 2 基础与机器人仿真（ch01-ch21，标准 ROS 2 包）**

| 章节 | 包名 | 说明 |
|------|------|------|
| Ch01 | `lifecycle_demo` | 生命周期节点、`/cmd_vel` 发布 |
| Ch02 | `hello_pkg` | 节点创建、日志分级 |
| Ch03 | `topic_demo` | 话题发布/订阅/QoS/正方形轨迹 |
| Ch03 | `sensor_pub` | 自定义 SensorData 消息发布 |
| Ch03 | `sensor_interfaces` | SensorData.msg 定义 |
| Ch04 | `service_demo` | AddTwoInts 服务 |
| Ch05 | `action_demo` | DoDishes 动作（异步执行） |
| Ch06 | `param_demo` | 参数声明/Launch 配置 |
| Ch07 | `tf_demo` | TF2 广播/监听 |
| Ch08 | `urdf_demo` | URDF/Xacro 建模 + RViz |
| Ch09 | `sim_demo` | Gazebo 仿真启动（委托 `robot_sim_demo`） |
| Ch10 | `slam_lab` | SLAM/Cartographer/AMCL |
| Ch11 | `navigation_lab` | Nav2 导航/航点/恢复 |
| Ch12 | `realsense_lab` | RealSense 深度相机启动 |
| Ch13 | `slam_bringup_lab` | SLAM 一键建图（`robot_sim_demo` + slam_toolbox） |
| Ch14 | `nav_bringup_lab` | Nav2 一键导航（`robot_sim_demo` + Nav2） |
| Ch15 | `arm_joint_pub_lab` | xArm 关节状态发布 |
| Ch17 | `moveit_fk_ik_lab` | MoveIt FK/IK 规划（MoveItPy） |
| Ch18 | `moveit_pick_place_lab` | MoveIt 抓取/避障/附着演示 |
| Ch19 | `vision_detection_lab` | 相机/cv_bridge/颜色/AR 码检测 |
| Ch21 | `vision_pickup_lab` | 视觉引导抓取（AR + xArm） |

Ch16、Ch20 为占位章节（仅 README）。

**CARLA 自动驾驶实验（ch22-ch31，实验脚本，配合 av_* 包）**

| 章节 | 脚本 | 说明 |
|------|------|------|
| Ch22 | `explore_carla.py`、`spawn_vehicles.py` | CARLA 仿真器入门：环境探索与车辆生成 |
| Ch23 | `spawn_ego.py`、`bridge_launch.sh` | CARLA-ROS2 桥接与 ego 车辆生成、话题检查 |
| Ch24 | `sensor_config.py`、`record_bag.sh` | 多传感器配置、数据采集（rosbag）与可视化 |
| Ch25 | `global_planner.py`、`waypoint_pub.py` | 全局路径规划（A*）、地图可视化与航点发布 |
| Ch26 | `pid_controller.py`、`pure_pursuit.py` | 车身控制：纵向 PID、横向 Pure Pursuit、一体化控制 |
| Ch27 | `localization_eval.py`、`tf_broadcaster.py` | 多传感器融合定位（EKF）与定位评估 |
| Ch28 | `yolo_detector.py`、`lidar_cluster.py` | 目标检测与跟踪：YOLO 检测、LiDAR 聚类、目标跟踪 |
| Ch29 | `fsm_decision.py`、`traffic_light_detector.py` | 行为决策（FSM）与交通灯感知 |
| Ch30 | `safety_monitor.py`、`fault_injector.py` | 安全验证：TTC 监控、故障注入、集成测试、评估指标 |
| Ch31 | `main_pipeline.py`、`town_demo.sh` | 综合项目：全流程自动驾驶 Pipeline 与一键演示 |

lab_code 详细说明见 [lab_code/README.md](src/lab_code/README.md)。

## 快速开始

### 1. 启动仿真

```bash

ros2 launch robot_sim_demo gazebo2.launch.py
```

Gazebo 打开后，Wheeltec Mini AKM 机器人在 ISCAS Museum 场景中心自动巡航。

### 2. 检查话题

```bash
ros2 topic list
ros2 topic echo /clock --once
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

### 3. 运行 SLAM 建图

```bash
# 终端 1
ros2 launch robot_sim_demo gazebo2.launch.py gui:=false rviz:=false drive:=false

# 终端 2
ros2 launch slam_sim_demo_ros2 slam_demo.launch.py use_gazebo:=false use_rviz:=true

# 终端 3
ros2 run slam_sim_demo_ros2 slam_map_runner --ros-args -p use_sim_time:=true
```

### 4. 运行 Nav2 导航

```bash
# 终端 1
ros2 launch robot_sim_demo gazebo2.launch.py gui:=false rviz:=false drive:=false

# 终端 2
ros2 launch navigation_sim_demo_ros2 nav2_demo.launch.py use_gazebo:=false use_rviz:=true

# 终端 3
ros2 run navigation_sim_demo_ros2 nav_goal_runner --ros-args -p use_sim_time:=true -p goal_x:=1.0 -p goal_y:=0.0
```

## 测试

仓库级测试命令会覆盖核心包、实验包和 CARLA Python 包；`course_lab_utils` 的部分测试需 `moveit_py`：

```bash

python3 -m pytest -q
# 当前无 ROS 2 的静态测试环境：189 passed, 43 subtests passed

# CARLA 自动驾驶 Python 包
for pkg in av_carla_interfaces av_sensor_kit av_perception_py av_planning_py av_safety_monitor; do

  echo "=== $pkg ==="

  (cd src/$pkg && python3 -m pytest test/ -q)

done

# 核心与示例包
for pkg in action_demo_py topic_demo_py service_demo_py param_demo_py tf_demo_py \

  tf_follower_ros2 navigation_sim_demo_ros2 slam_sim_demo_ros2 robot_sim_demo \

  urdf_demo_ros2 xarm; do

  echo "=== $pkg ==="

  (cd src/$pkg && python3 -m pytest test/ -q)

done

# lab_code 各章（详见 src/lab_code/README.md）
for pkg in ch01_lab ch02_lab/hello_pkg ch03_lab/sensor_pub ch03_lab/topic_demo \

  ch04_lab/service_demo ch05_lab/action_demo ch06_lab/param_demo ch07_lab/tf_demo \

  ch08_lab/urdf_demo ch09_lab/sim_demo ch10_lab/slam_lab ch11_lab/navigation_lab \

  ch12_lab/realsense_lab ch13_lab/slam_bringup_lab ch14_lab/nav_bringup_lab \

  ch15_lab/arm_joint_pub_lab ch17_lab/moveit_fk_ik_lab ch18_lab/moveit_pick_place_lab \

  ch19_lab/vision_detection_lab ch21_lab/vision_pickup_lab; do

  echo "=== $pkg ==="

  (cd src/lab_code/$pkg && python3 -m pytest test/ -q)

done
```

### 仿真核心与支持包

| 包名 | 测试数 | 结果 |
|------|--------|------|
| `robot_sim_demo` | 8 | 全通过 |
| `xarm_ros2_arm_only` | 9 | 全通过 |
| `tf_follower_ros2` | 7 | 全通过 |
| `slam_sim_demo_ros2` | 6 | 全通过 |
| `navigation_sim_demo_ros2` | 5 | 全通过 |
| `urdf_demo_ros2` | 1 | 通过 |

### 示例包

| 包名 | 测试数 | 结果 |
|------|--------|------|
| `action_demo_py` | 1 | 通过 |
| `topic_demo_py` | 1 | 通过 |
| `service_demo_py` | 1 | 通过 |
| `param_demo_py` | 1 | 通过 |
| `tf_demo_py` | 1 | 通过 |

### lab_code 教学实验包

| 包名 | 测试数 | 结果 |
|------|--------|------|
| `lifecycle_demo` | 1 | 通过 |
| `hello_pkg` | 2 | 通过 |
| `sensor_pub` | 1 | 通过 |
| `topic_demo` | 2 | 通过 |
| `service_demo` | 2 | 通过 |
| `action_demo` | 3 | 通过 |
| `param_demo` | 5 | 通过 |
| `tf_demo` | 4 | 通过 |
| `urdf_demo` | 5 | 通过 |
| `sim_demo` | 4 | 通过 |
| `slam_lab` | 5 | 通过 |
| `navigation_lab` | 6 | 通过 |
| `realsense_lab` | 3 | 通过 |
| `slam_bringup_lab` | 5 | 通过 |
| `nav_bringup_lab` | 4 | 通过 |
| `arm_joint_pub_lab` | 5 | 通过 |
| `moveit_fk_ik_lab` | 6 | 通过 |
| `moveit_pick_place_lab` | 3 | 通过 |
| `vision_detection_lab` | 4 | 通过 |
| `vision_pickup_lab` | 5 | 通过 |

运行截图和预期现象见各章节实验手册；当前源码目录不包含固定的 Nav2 GIF 资源。

## 嵌套包说明

`src/robot_sim_demo/wheeltec_robot_urdf/` 是一个独立的 ament_cmake 包，包含 Wheeltec 机器人的 URDF 和 STL 网格资源。由于它嵌套在 `robot_sim_demo` 包内部，colcon 不会单独发现它，而是作为 `robot_sim_demo` 的数据文件安装到 `share/robot_sim_demo/wheeltec_robot_urdf/`。
