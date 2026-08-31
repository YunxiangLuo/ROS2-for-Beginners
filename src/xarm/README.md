# xArm6 纯机械臂 ROS 2 仿真

此目录是从父项目的 xArm 仿真中派生出的独立 ROS 2 软件包。它仅在 Gazebo Harmonic 中启动 xArm6 机械臂，并连接到 ros2_control、MoveIt 2 和 RViz。

Gazebo 世界有意不包含 `<model>`、`<include>` 或传感器元素。父项目中的操作台、底座、灶台、饭盒、食材、RGB-D 相机、厨房节点以及厨房 RViz 标记均未使用。定向光和 Gazebo 系统插件属于世界基础设施，而非场景对象。xArm 会在运行时从 `/robot_description` 生成，并直接固定到 `world` 坐标系。

## 软件包结构

```text

xarm_ros2_arm_only/

  config/    ros2_control、MoveIt、RViz 和 Gazebo GUI 配置

  launch/    完整仿真和 move_group 启动文件

  resource/  ament 软件包索引标记

  test/      源码级隔离和配置检查

  urdf/      xArm Gazebo 封装 Xacro

  worlds/    空白纯机械臂 Gazebo 世界
```

## 运行环境要求

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- `ros_gz`、`gz_ros2_control`、ros2_control、MoveIt 2 和 TRAC-IK
- 本仓库提供的 Jazzy 兼容 `xarm_description` 软件包

本软件包使用本仓库 `src/xarm_description/` 中的 **XBot Arm** `xarm_description` 版本 `2.0.0`。它提供 `arm_1_joint` 至 `arm_6_joint`、`gripper_1_joint`、`gripper_2_joint` 和 `gripper_centor_link`，以及以下文件和其引用的网格资源：

```text
xarm_description/urdf/arm.urdf.xacro
```

此接口与使用 `joint1` 至 `joint6` 等关节名的当前 UFACTORY 官方描述软件包不兼容。除非同时迁移 Xacro、SRDF、控制器和 MoveIt 配置，否则不要用该官方软件包替换父项目的底层软件包。

在 source ROS 2 工作区后，检查所需软件包：

```bash

ros2 pkg prefix xarm_description

ros2 pkg prefix gz_ros2_control

ros2 pkg prefix moveit_ros_move_group

ros2 pkg prefix moveit_configs_utils

ros2 pkg prefix trac_ik_kinematics_plugin
```

## 构建

请在工作区根目录执行命令。`xarm_ros2_arm_only` 位于 `src/xarm/`，可被 colcon 直接发现并作为工作区的一个软件包构建。

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/ROS2
colcon build --base-paths src --symlink-install \
  --packages-select xarm_description xarm_ros2_arm_only
source install/setup.bash
```

完整构建整个工作区时，使用 `colcon build --base-paths src --symlink-install`；课程源码当前发现 46 个可构建包，其中 `lab_code/ch16_lab/` 是不带 `package.xml` 的纯文件示例。xArm 的运行和 MoveIt 验证使用同一工作区中的 `xarm_description`。

不要复用其他机器上的 `build/`、`install/` 或 `log/` 目录。

## 启动

同时启动 Gazebo、ros2_control、MoveIt 和 RViz：

```bash

ros2 launch xarm_ros2_arm_only arm_only.launch.py
```

RViz 会以 MoveIt 的 MotionPlanning 显示配置打开。选择规划组 `xarm`，拖动目标状态标记，然后使用 `Plan` 或 `Plan & Execute`。

常用启动变体：

```bash
# 仅运行 Gazebo 服务端，启用 MoveIt，不启动 RViz
ros2 launch xarm_ros2_arm_only arm_only.launch.py \
  gz_headless:=true use_rviz:=false

# 仅启动机器人描述和 MoveIt；不使用 Gazebo /clock，改用墙钟时间
ros2 launch xarm_ros2_arm_only arm_only.launch.py \
  use_gazebo:=false use_sim_time:=false

# 在不添加底座模型的前提下抬高固定机械臂底座
ros2 launch xarm_ros2_arm_only arm_only.launch.py base_height:=0.20
```

`world` 和 `world_name` 启动参数必须指向同一个 Gazebo 世界。其默认值分别为 `worlds/arm_only.sdf` 和 `arm_only`。

## 验证

启动前验证源码资源：

```bash

python3 -m pytest test -q

xacro urdf/arm_only_xarm.urdf.xacro \

  hardware_type:=gz \

  use_ros2_control:=true \

  controllers_file:=$PWD/config/arm_only_controllers.yaml \

  include_world_joint:=true \

  base_height:=0.0 > /tmp/arm_only_xarm.urdf

check_urdf /tmp/arm_only_xarm.urdf

gz sdf -k worlds/arm_only.sdf
```

启动后检查活动控制器和 MoveIt 服务：

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 service list | grep -E 'compute_ik|plan_kinematic_path'
```

在第二个已 source 环境的终端中运行随软件包提供的端到端检查。它会验证三个活动控制器、`/joint_states` 和一次 OMPL 规划：

```bash

ros2 run xarm_ros2_arm_only arm_only_runtime_smoke
```

若要通过 MoveIt 和 Gazebo 控制器执行其简短的关节空间测试轨迹：

```bash
ros2 run xarm_ros2_arm_only arm_only_runtime_smoke --execute
```

预期的控制器如下：

```text

joint_state_broadcaster

xarm_controller

gripper_controller
```

## 预期运行消息

由于此软件包有意不包含相机或感知场景，MoveIt 可能报告未配置 3D 传感器插件，RViz 可能报告可选的 `/recognize_objects` 动作不可用。这些消息不会影响关节空间规划或执行。

Gazebo 的 ODE 后端也可能提示无法为 `gripper_2_joint` 创建原生 mimic 约束。本地 ros2_control 描述会将该关节注册为 `gripper_1_joint` 的 mimic 关节，且 `gz_ros2_control` 会在启动时报告此映射。

本软件包仅用于仿真，不配置或控制物理硬件。
