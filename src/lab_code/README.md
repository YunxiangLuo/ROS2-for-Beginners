# src/lab_code/ — ROS 2 教学实验代码

本目录包含 ROS 2 Jazzy 教学课程的实验代码，覆盖节点、话题、服务、动作、参数、

TF、URDF、Gazebo 仿真、SLAM、Nav2、MoveIt 2、视觉与 CARLA 自动驾驶等主题。

共 **31 个章节**（ch01-ch31）：

- **ch01-ch21**：ROS 2 基础与机器人仿真（以 ROS 2 包为主；ch16 为纯文件示例，ch20 为设计说明占位）
- **ch22-ch31**：CARLA 自动驾驶实验（实验脚本，配合 `src/` 下 av_* 包）

## 环境

| 组件 | 版本 |
|------|------|
| 操作系统 | Ubuntu 24.04 (WSL2) |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Sim Harmonic (v8) |
| CARLA | 0.9.16（ch22-ch31） |
| 构建工具 | colcon + ament |

## 仿真分工

| 仿真场景 | 对应包/工具 | 用于章节 |
|----------|-------------|----------|
| 移动机器人仿真 | `robot_sim_demo`（Wheeltec + ISCAS Museum） | ch09-ch11、ch13-ch14 |
| xArm6 机械臂仿真 | `xarm_ros2_arm_only`（Gazebo + MoveIt 2 + RViz） | ch15、ch17-ch18、ch21 |
| CARLA 自动驾驶 | CARLA 0.9.16 + `carla_ros_bridge` + av_* 包 | ch22-ch31 |

`robot_sim_demo` 与 `xarm_ros2_arm_only` 均位于 `src/` 下，单独构建即可。

视觉章节（ch19、ch21）的相机输入可来自 `robot_sim_demo` 的 `/camera/image_raw`

或真实 USB/RealSense 相机。CARLA 章节需先安装 CARLA 仿真器（见

[ch22_lab/install_carla.sh](ch22_lab/install_carla.sh)）并启动

`carla_ros_bridge`。

## 章节清单与测试状态

### ROS 2 基础与机器人仿真（ch01-ch21）

| 章节 | 包名 | 内容 | 测试 | 结果 |
|------|------|------|------|------|
| ch01 | `lifecycle_demo` | 生命周期节点、`/cmd_vel` | 1 | ✅ |
| ch02 | `hello_pkg` | 节点、日志、里程计监听 | 2 | ✅ |
| ch03 | `sensor_interfaces` | `SensorData.msg` 接口 | — | ✅ 构建 |
| ch03 | `sensor_pub` | 自定义消息发布 | 1 | ✅ |
| ch03 | `topic_demo` | 话题/QoS/正方形轨迹 | 2 | ✅ |
| ch04 | `service_demo` | AddTwoInts 服务 | 2 | ✅ |
| ch05 | `action_demo` | DoDishes 动作 | 3 | ✅ |
| ch06 | `param_demo` | 参数声明/回调/Launch | 5 | ✅ |
| ch07 | `tf_demo` | TF2 广播/监听 | 4 | ✅ |
| ch08 | `urdf_demo` | URDF/Xacro + RViz | 5 | ✅ |
| ch09 | `sim_demo` | 委托 `robot_sim_demo` Gazebo 入口 | 4 | ✅ |
| ch10 | `slam_lab` | SLAM Toolbox/Cartographer/AMCL | 5 | ✅ |
| ch11 | `navigation_lab` | Nav2 目标/航点/恢复/监控 | 6 | ✅ |
| ch12 | `realsense_lab` | RealSense 相机启动 | 3 | ✅ |
| ch13 | `slam_bringup_lab` | SLAM 一键建图（委托核心包） | 5 | ✅ |
| ch14 | `nav_bringup_lab` | Nav2 一键导航（委托核心包） | 4 | ✅ |
| ch15 | `arm_joint_pub_lab` | xArm 关节状态发布 | 5 | ✅ |
| ch16 | — | 纯文件示例（URDF/Xacro 与状态发布脚本，非 ROS 2 包） | — | — |
| ch17 | `moveit_fk_ik_lab` | MoveIt FK/IK 规划 | 6 | ✅ |
| ch18 | `moveit_pick_place_lab` | MoveIt 抓取放置/避障/附着 | 3 | ✅ |
| ch19 | `vision_detection_lab` | 相机/cv_bridge/颜色/AR 检测 | 4 | ✅ |
| ch20 | — | 设计说明占位（无实现包） | — | — |
| ch21 | `vision_pickup_lab` | 视觉引导抓取（AR + xArm） | 5 | ✅ |

**ch01-ch21 合计 75 项测试全部通过。**

### CARLA 自动驾驶实验（ch22-ch31，实验脚本）

| 章节 | 脚本 | 内容 | 测试 |
|------|------|------|------|
| ch22 | `explore_carla.py`、`spawn_vehicles.py` | CARLA 仿真器入门：环境探索、地图信息、多车生成 | — |
| ch23 | `spawn_ego.py`、`check_topics.py` | CARLA-ROS2 桥接与 Ego 车辆生成、话题检查 | — |
| ch24 | `sensor_config.py`、`visualize_sensors.py` | 多传感器配置、数据采集（rosbag）与可视化 | — |
| ch25 | `global_planner.py`、`waypoint_pub.py` | 全局路径规划（A*）、地图可视化、航点发布 | — |
| ch26 | `pid_controller.py`、`pure_pursuit.py` | 纵向 PID、横向 Pure Pursuit、一体化车辆控制 | — |
| ch27 | `localization_eval.py`、`tf_broadcaster.py` | LiDAR-IMU-GNSS 融合定位（EKF）与评估 | — |
| ch28 | `yolo_detector.py`、`lidar_cluster.py` | YOLO 目标检测、LiDAR DBSCAN 聚类、卡尔曼跟踪 | — |
| ch29 | `fsm_decision.py`、`traffic_light_detector.py` | 行为决策（FSM：巡航/跟车/停车）与交通灯感知 | — |
| ch30 | `safety_monitor.py`、`fault_injector.py` | 安全验证：碰撞监测/AEB、故障注入、集成测试、评估指标 | — |
| ch31 | `main_pipeline.py`、`town_demo.sh` | 综合项目：CARLA Town03 全流程自动驾驶演示 | — |

> ch22-ch31 为裸 Python/Shell 实验脚本（非 ROS 2 包），直接 `python3` /
> `bash` 运行；规范化实现由 `src/` 下的 av_* 包提供（`av_carla_interfaces`、
> `av_sensor_kit`、`av_perception_py`、`av_planning_py`、`av_control_cpp`、
> `av_safety_monitor`），部分脚本支持 `--test` 自检。

## 构建

ch01-ch21 以 ROS 2 包为主；ch16 为纯文件示例、ch20 为设计说明占位，二者不参与 colcon 构建。在工作区根目录构建其余功能包：

```bash

source /opt/ros/jazzy/setup.bash
# 构建机械臂相关章节前，需 source 兼容的 xarm_description 底层工作区
cd <robot_sim_demo 工作区>

colcon build --symlink-install

source install/setup.bash
```

单独构建某一章，例如：

```bash
colcon build --symlink-install --packages-select realsense_lab
```

> 说明：ch17/ch18/ch21 依赖 `course_lab_utils` 与 `course_lab_interfaces`
> >（位于 `src/` 下），colcon 会自动按依赖顺序构建。

CARLA 章节（ch22-ch31）无需 colcon 构建，按需安装依赖：

```bash
# CARLA 仿真器（一键安装见 ch22_lab/install_carla.sh）
cd ~/carla && ./CarlaUE4.sh -quality-level=Low

# Python 依赖
pip install carla numpy opencv-python

pip install ultralytics scikit-learn filterpy   # ch28 可选
```

## 测试

```bash
for pkg in lifecycle_demo hello_pkg sensor_pub topic_demo service_demo \
           action_demo param_demo tf_demo urdf_demo sim_demo slam_lab \
           navigation_lab realsense_lab slam_bringup_lab nav_bringup_lab \
           arm_joint_pub_lab moveit_fk_ik_lab moveit_pick_place_lab \
           vision_detection_lab vision_pickup_lab; do
  colcon test --packages-select $pkg
done
colcon test-result --all
```

CARLA 控制算法自检（无需仿真器）：

```bash

cd src/lab_code/ch26_lab && python3 pid_controller.py --test && python3 pure_pursuit.py --test
```

## 目录结构

```text
src/lab_code/
├── ch01_lab/lifecycle_demo/          ✅ 生命周期节点
├── ch02_lab/hello_pkg/               ✅ 节点/日志/里程计
├── ch03_lab/{sensor_interfaces,sensor_pub,topic_demo}/  ✅ 消息与话题
├── ch04_lab/service_demo/            ✅ 服务
├── ch05_lab/action_demo/             ✅ 动作
├── ch06_lab/param_demo/              ✅ 参数
├── ch07_lab/tf_demo/                 ✅ TF2
├── ch08_lab/urdf_demo/               ✅ URDF/Xacro + RViz
├── ch09_lab/sim_demo/                ✅ Gazebo（委托 robot_sim_demo）
├── ch10_lab/slam_lab/                ✅ SLAM/AMCL
├── ch11_lab/navigation_lab/          ✅ Nav2
├── ch12_lab/realsense_lab/           ✅ RealSense 相机
├── ch13_lab/slam_bringup_lab/        ✅ SLAM 一键建图
├── ch14_lab/nav_bringup_lab/         ✅ Nav2 一键导航
├── ch15_lab/arm_joint_pub_lab/       ✅ xArm 关节发布
├── ch16_lab/                          纯文件示例（非 ROS 2 包）
├── ch17_lab/moveit_fk_ik_lab/        ✅ MoveIt FK/IK
├── ch18_lab/moveit_pick_place_lab/   ✅ MoveIt 抓取放置
├── ch19_lab/vision_detection_lab/    ✅ 视觉检测
├── ch20_lab/                          设计说明占位（无实现包）
├── ch21_lab/vision_pickup_lab/       ✅ 视觉引导抓取
├── ch22_lab/                          CARLA 入门与车辆生成（脚本）
├── ch23_lab/                          CARLA-ROS2 桥接与 Ego 生成（脚本）
├── ch24_lab/                          多传感器配置与数据采集（脚本）
├── ch25_lab/                          全局路径规划与航点（脚本）
├── ch26_lab/                          车身控制 PID/Pure Pursuit（脚本）
├── ch27_lab/                          多传感器融合定位 EKF（脚本）
├── ch28_lab/                          目标检测与跟踪（脚本）
├── ch29_lab/                          行为决策与交通灯（脚本）
├── ch30_lab/                          安全验证与故障注入（脚本）
├── ch31_lab/                          综合自动驾驶项目（脚本）
└── README.md                          本文件
```

## 运行结果截图

各章节 README 的「运行结果」小节说明了预期现象与截图保存路径（一般为

`docs/images/`）。截图需在实际启动仿真或节点后捕获；核心通信章节可在终端

直接观察日志输出作为运行证据。
