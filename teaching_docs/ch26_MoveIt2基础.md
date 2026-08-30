# 第26章 MoveIt2基础

> **课程**：ROS2 Python 编程  
> **章节**：第26章  
> **课时**：2 课时（90 分钟）  
> **教学方式**：讲授 + 演示  

---

## 学习目标

本章学习目标包括：理解MoveIt2的架构设计，掌握Setup Assistant配置流程，学会使用MoveGroupInterface API，理解OMPL运动规划器的工作原理，掌握碰撞检测的配置和使用。

## 26.1 MoveIt2架构

### 26.1.1 MoveIt2概述

MoveIt2是ROS2环境下最主流的机械臂运动规划框架，是ROS1 MoveIt的升级版本。它综合了运动规划、控制、3D感知、运动学和控制的最新成果，提供了开发先进机器人应用的易用平台。

```
┌──────────────────────────────────────────────────────┐
│                    MoveIt2 架构                        │
│                                                        │
│  ┌──────────┐    ┌──────────────────────────────┐     │
│  │ 用户接口  │───>│      move_group 节点         │     │
│  │(C++/Python)│   │  (核心集成器)                │     │
│  └──────────┘    └──────────────┬───────────────┘     │
│                                  │                      │
│        ┌─────────────────────────┼──────────────┐     │
│        │                         │              │     │
│  ┌─────▼──────┐    ┌─────────────▼─────┐  ┌────▼───┐ │
│  │ 运动规划器   │    │  规划场景监听器    │  │控制器  │ │
│  │ OMPL/STOMP  │    │  PlanningScene    │  │Manager │ │
│  └─────┬──────┘    └─────────────┬─────┘  └────┬───┘ │
│        │                         │              │      │
│  ┌─────▼─────────────────────────▼──────────────▼──┐  │
│  │           ros2_control (硬件抽象层)              │  │
│  │   FollowJointTrajectory / JointState 接口       │  │
│  └──────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────┘
```

### 26.1.2 核心模块

**move_group节点**：

move_group是MoveIt2的核心集成器，将所有系统组件组合在一起，提供一系列可供用户自由使用的ROS2操作与服务。

常用的使用方式有三种：C++ API由`move_group_interface`包提供C++接口，Python API由`moveit_py`包提供Python接口，GUI则基于RViz2中的MotionPlanning插件。

**配置参数**：

| 参数 | 说明 | 来源 |
|------|------|------|
| robot_description | URDF机器人模型 | 参数服务器 |
| robot_description_semantic | SRDF语义模型 | 参数服务器 |
| kinematics.yaml | 运动学求解器配置 | MoveIt配置包 |
| joint_limits.yaml | 关节限制配置 | MoveIt配置包 |
| ompl_planning.yaml | OMPL规划器配置 | MoveIt配置包 |

### 26.1.3 规划场景（PlanningScene）

PlanningScene是MoveIt2中环境模型的核心概念，包含：

其中**RobotState**表示当前机器人状态（各关节位置），**CollisionObjects**是环境中的碰撞物体，**AttachedCollisionObjects**是附着在机器人上的物体，**OccupancyMap**则是环境占用地图。

### 26.1.4 运动规划流程

MoveIt2的典型运动规划流程：

```
用户设定目标 → move_group接收请求
                  → 查询规划场景（当前状态+环境）
                  → 调用运动学求解器（IK/FK）
                  → 调用运动规划器（OMPL等）
                  → 碰撞检测验证
                  → 返回规划轨迹
                  → 通过控制器执行轨迹
```

### 26.1.5 官方要点——MoveIt 官方文档：move_group 中枢与规划流水线

> 本节内容综合翻译自 MoveIt 2 官方文档（moveit.picknik.ai 的 Concepcepts、MoveItPy 与 Collision Checking 页面）、OMPL 官方文档（ompl.kavrakilab.org）以及 PickNik Academy 官方培训课程，另参考 The Construct 的 MoveIt 课程与 Robotics Back-End 的规划器对比教程。原文均为英文，此处为中文编译，供课后巩固与进阶阅读。

MoveIt 官方 Concepts 文档将 move_group 描述为系统唯一"上帝节点"：它持有机器人模型（robot model）、规划场景（planning scene）、规划流水线（planning pipeline）与控制器管理器（controller manager），对外只暴露 ROS 2 接口（action、service、topic）——这正是本章图 26.1 的结构。官方文档强调流水线级联机制：`planning_adapters`（如 FixStartStateBounds、AddTimeParameterization）先修正请求再交给规划器（planner），规划结果再做时间参数化，因此 OMPL 输出的是"关节路径"，时间参数化才赋予速度/加速度——练习第 4 题的含时路径本质上来自这一层。

## 26.2 MoveIt2 Setup Assistant

### 26.2.1 安装MoveIt2

```bash
# 安装MoveIt2完整包
sudo apt install ros-jazzy-moveit

# 安装MoveIt2 Python API
sudo apt install ros-jazzy-moveit-py

# 安装Setup Assistant
sudo apt install ros-jazzy-moveit-setup-assistant
```

### 26.2.2 启动配置助手

```bash
# 启动Setup Assistant
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

### 26.2.3 配置步骤详解

**步骤1：加载机器人模型**

点击"Create New MoveIt Configuration Package"，选择URDF或Xacro文件加载。系统会自动解析机器人结构。

**步骤2：生成自碰撞免检矩阵**

点击"Generate Collision Matrix"，系统自动检测不可能发生碰撞的连杆对，将其加入免检列表，减少规划计算量。

```
自动碰撞矩阵示例：
base_link ↔ link2: 距离远，永不碰撞 → 免检
link1 ↔ link2: 相邻连杆，碰撞检测 → 需检
link2 ↔ link4: 距离远，永不碰撞 → 免检
```

**步骤3：添加虚拟关节（可选）**

用于将机器人基座标系连接到世界坐标系：

```xml
<virtual_joint name="base_footprint"
               type="fixed"
               parent_frame="world"
               child_link="base_link"/>
```

**步骤4：添加规划组**

规划组在语义上描述机器人的不同部分：

```yaml
# kinematics.yaml
arm_group:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 3
```

规划组的配置方式有两种：基于关节（显式列出组包含的关节），或基于连杆（按连杆链自动确定关节）。

**步骤5：添加末端执行器**

指定手爪规划组为特定的末端执行器：

```xml
<end_effector name="gripper_ee"
              parent_link="link5"
              parent_group="arm_group"
              group="gripper"/>
```

**步骤6：添加预设位姿**

```xml
<group_state name="home" group="arm_group">
    <joint name="joint1" value="0"/>
    <joint name="joint2" value="0"/>
    <joint name="joint3" value="0"/>
</group_state>

<group_state name="retract" group="arm_group">
    <joint name="joint1" value="0"/>
    <joint name="joint2" value="-1.57"/>
    <joint name="joint3" value="1.57"/>
</group_state>
```

**步骤7：ROS2 Control配置**

```yaml
controller_list:
  - name: arm_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
  - name: gripper_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    joints:
      - gripper_left_joint
      - gripper_right_joint
```

**步骤8：生成配置包**

选择输出目录，点击"Generate Package"生成MoveIt2配置功能包。

### 26.2.4 配置文件解析

生成的配置包目录结构：

```
generated_moveit_config/
├── config/
│   ├── arm.srdf                # 语义描述
│   ├── kinematics.yaml         # 运动学配置
│   ├── joint_limits.yaml       # 关节极限
│   ├── ompl_planning.yaml      # OMPL规划器
│   ├── fake_controllers.yaml   # 虚拟控制器
│   └── ros2_controllers.yaml   # 实际控制器
├── launch/
│   ├── move_group.launch.py    # move_group启动
│   ├── planning_context.launch.py  # 模型加载
│   ├── demo.launch.py          # 演示模式
│   └── setup_assistant.launch.py
└── rviz/
    └── moveit.rviz             # Rviz配置
```

**joint_limits.yaml**：

```yaml
joint_limits:
  joint1:
    has_velocity_limits: true
    max_velocity: 0.5
    has_acceleration_limits: true
    max_acceleration: 0.5
  joint2:
    has_velocity_limits: true
    max_velocity: 0.5
    has_acceleration_limits: true
    max_acceleration: 0.5
```

**fake_controllers.yaml**：

```yaml
controller_list:
  - name: fake_arm_controller
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
```

## 26.3 MoveGroupInterface

### 26.3.1 基本概念

MoveGroupInterface是MoveIt2提供给用户的主要编程接口，支持C++和Python两种语言。它封装了运动规划、执行、运动学求解、碰撞检测等功能。

### 26.3.2 Python中的MoveGroupInterface

```python
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningComponent

class MoveGroupDemo(Node):
    def __init__(self):
        super().__init__('move_group_demo')

        # 初始化MoveItPy
        self.moveit = MoveItPy(node_name='moveit_py')

        # 获取规划组件
        self.arm = PlanningComponent(
            self.moveit, 'arm_group', 'link5'
        )

        # 配置规划参数
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.01)
        self.arm.set_max_velocity_scaling_factor(0.5)
        self.arm.set_max_acceleration_scaling_factor(0.5)

        self.get_logger().info('MoveGroupInterface 初始化完成')

    def plan_to_joint_target(self, joint_values):
        """规划到关节目标"""
        self.arm.set_start_state_to_current_state()
        self.arm.set_joint_value_target(joint_values)
        plan_result = self.arm.plan()
        if plan_result:
            self.get_logger().info('规划成功，开始执行')
            self.arm.execute(plan_result.trajectory)
        else:
            self.get_logger().error('规划失败')

    def plan_to_pose_target(self, pose):
        """规划到位姿目标"""
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(pose, 'link5')
        plan_result = self.arm.plan()
        if plan_result:
            self.arm.execute(plan_result.trajectory)
```

### 26.3.3 C++中的MoveGroupInterface

```cpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("move_group_demo");

    // 创建MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group(
        node, "arm_group");

    // 设置关节目标
    move_group.setJointValueTarget({0.0, 0.5, -0.5, 0.0, 0.0, 0.0});

    // 规划并执行
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        move_group.execute(plan);
    }

    rclcpp::shutdown();
    return 0;
}
```

### 26.3.4 常用API方法

| 方法 | 功能 | 参数 |
|------|------|------|
| set_joint_value_target() | 设置关节目标 | 关节角度列表 |
| set_pose_target() | 设置末端位姿目标 | Pose对象/Stamped |
| set_named_target() | 设置命名位姿 | 位姿名称字符串 |
| set_start_state_to_current_state() | 设置起始状态为当前 | 无 |
| plan() | 规划轨迹 | 可选规划器ID |
| execute() | 执行轨迹 | 规划结果 |
| get_current_pose() | 获取当前位姿 | 末端link名称 |
| get_current_joint_values() | 获取当前关节值 | 无 |
| set_goal_tolerance() | 设置容忍度 | 位置+姿态容忍度 |
| set_max_velocity_scaling_factor() | 速度缩放 | 0.0~1.0 |
| set_max_acceleration_scaling_factor() | 加速度缩放 | 0.0~1.0 |

### 26.3.5 官方要点——官方引领的 Python 路线：MoveItPy 教程

MoveIt 官方文档在"Python 或许更简单"的工程命题下重点介绍了 MoveItPy：它绕开 move_group 进程，直接在应用进程内构造 `PlanningSceneMonitor` 与 `MoveItPy` 对象，用 `PlanComponents` 组装规划请求，`plan()` 返回带轨迹的规划结果，`execute()` 走 FollowJointTrajectory action。官方教程的典型结构——加载 robot description、定义初始姿态、多次随机查询并统计规划时间——与练习第 2 题逐行对应；它还可以切换到离线"免状态监视"模式，适合练手与单元测试。

## 26.4 OMPL运动规划器

### 26.4.1 OMPL概述

OMPL（Open Motion Planning Library）是MoveIt2默认的运动规划算法库，包含多种基于采样的规划算法。这些算法在高维配置空间中搜索可行路径。

### 26.4.2 主要规划算法

**RRT（Rapidly-exploring Random Tree）**：

从起点开始，随机采样并扩展树状结构，直到到达目标区域。RRTConnect在起点和终点同时构建两棵树，加速收敛。

```
RRT算法流程：
1. 初始化树T，包含起始节点q_start
2. 循环直到到达目标或超时：
   a. 随机采样q_rand
   b. 找到T中最近的节点q_near
   c. 沿q_near→q_rand方向扩展一步得到q_new
   d. 检查q_near→q_new路径是否无碰撞
   e. 若无碰撞，将q_new加入T
   f. 若q_new接近目标，返回路径
```

**其他规划算法**：

| 算法 | 特点 | 适用场景 |
|------|------|---------|
| RRTConnect | 双向RRT，收敛快 | 通用场景 |
| PRM* | 渐进最优 | 多查询场景 |
| RRT* | 渐进最优RRT | 需要最优路径 |
| EST | 基于扩展空间树 | 狭窄通道 |
| KPIECE | 基于单元分割 | 高维空间 |

### 26.4.3 规划器配置

```yaml
# ompl_planning.yaml
arm_group:
  planner_configs:
    - RRTConnectkConfigDefault
    - RRTkConfigDefault
    - PRMkConfigDefault
    - ESTkConfigDefault

  # 默认规划器
  default_planner_config: RRTConnectkConfigDefault

RRTConnectkConfigDefault:
  type: geometric::RRTConnect
  range: 0.1            # 扩展步长
  goal_bias: 0.1        # 目标偏置概率
  max_planning_time: 1.0  # 最大规划时间
```

### 26.4.4 自定义规划参数

在Python代码中指定规划器和参数：

```python
from moveit.planning import MoveItPy, PlanningComponent

class CustomPlannerDemo(Node):
    def __init__(self):
        super().__init__('custom_planner_demo')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.arm = PlanningComponent(self.moveit, 'arm_group')

    def plan_with_specific_planner(self, joint_goal):
        self.arm.set_start_state_to_current_state()
        self.arm.set_joint_value_target(joint_goal)

        plan_result = self.arm.plan(
            planner_id='RRTConnectkConfigDefault',
            planning_time=10.0,  # 最大规划时间
            max_attempts=100,    # 最大尝试次数
        )

        if plan_result:
            self.arm.execute(plan_result.trajectory)
            return True
        return False
```

### 26.4.5 规划时间与质量权衡

| planning_time | 成功率 | 路径质量 | 适用场景 |
|:---:|:---:|:---:|:---:|
| 0.5s | 低 | 一般 | 快速运动 |
| 2.0s | 中 | 较好 | 常规任务 |
| 10.0s | 高 | 最优 | 精密操作 |

### 26.4.6 官方要点——OMPL 官方文档：三大规划器定位

OMPL 官方手册（Kavraki 实验室）对练习第 3 题给出的选择依据是：RRTConnect 是"双树连接"式单查询规划器，在工业 6 轴场景中成功率与速度的平衡最好、使用最广；RRT* 是渐进最优算法（Asymptotically Optimal），路径更短但收敛需要大量采样，实践中常配合限时截断使用；PRM 是多查询规划器，一次建图多次查询，适合固定环境的重复抓取任务。Nav2/MoveIt 侧的 ompl_planning.yaml 中，`planner_id` 决定具体算法，`max_planning_time` 与 `max_iterations` 是两个最影响成功率的超参，官方 benchmark 工具（ompl::benchmark）可量化对比三种算法的成功率-时间曲线。

## 26.5 碰撞检测

### 26.5.1 碰撞检测机制

MoveIt2使用FCL（Flexible Collision Library）和Bullet进行碰撞检测，支持三种碰撞检测模式：自碰撞检测（检查机器人连杆之间的碰撞）、环境碰撞检测（检查机器人与环境物体的碰撞）和附着物体碰撞检测（检查附着在机器人上的物体与环境碰撞）。

### 26.5.2 碰撞检测配置

**自碰撞免检矩阵**：

通过在SRDF中定义免检对减少计算量：

```xml
<disable_collisions link1="base_link" link2="link2" reason="Never"/>
<disable_collisions link1="base_link" link2="link3" reason="Never"/>
<disable_collisions link1="base_link" link2="link4" reason="Adjacent"/>
```

**碰撞检测设置**：

```yaml
# 在MoveIt配置中
collision_detection:
  type: "FCL"          # 碰撞引擎
  padding: 0.01        # 碰撞安全余量（米）
  scale: 1.0           # 碰撞缩放比例
```

### 26.5.3 编程中的碰撞检测

```python
from moveit.core.collision_detection import CollisionRequest, CollisionResult

class CollisionCheckDemo(Node):
    def __init__(self):
        super().__init__('collision_check_demo')
        self.moveit = MoveItPy(node_name='moveit_py')

    def check_collision(self, joint_positions):
        """检查给定关节位置是否碰撞"""
        robot_model = self.moveit.get_robot_model()
        planning_scene = self.moveit.get_planning_scene_monitor()

        # 设置机器人状态
        robot_state = robot_model.get_default_state()
        robot_state.set_joint_group_positions('arm_group', joint_positions)

        # 碰撞检测请求
        request = CollisionRequest()
        request.contacts = True
        request.max_contacts = 10

        # 执行检测
        result = CollisionResult()
        planning_scene.check_collision(request, result, robot_state)

        if result.collision:
            self.get_logger().warn('检测到碰撞！')
            for contact in result.contacts:
                self.get_logger().warn(
                    f'碰撞对: {contact.body1_name} ↔ {contact.body2_name}')
        else:
            self.get_logger().info('无碰撞')
```

### 26.5.4 添加碰撞物体

```python
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

class ObstacleManager:
    def __init__(self, planning_scene_monitor):
        self.psm = planning_scene_monitor

    def add_box(self, box_id, frame_id, x, y, z, size):
        """添加长方体障碍物"""
        co = CollisionObject()
        co.id = box_id
        co.header.frame_id = frame_id
        co.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = size

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        co.primitives = [primitive]
        co.primitive_poses = [pose.pose]

        self.psm.process_collision_object(co)

    def add_cylinder(self, cyl_id, frame_id, x, y, z, height, radius):
        """添加圆柱体障碍物"""
        co = CollisionObject()
        co.id = cyl_id
        co.header.frame_id = frame_id
        co.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [height, radius]

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        co.primitives = [primitive]
        co.primitive_poses = [pose.pose]

        self.psm.process_collision_object(co)

    def add_sphere(self, sphere_id, frame_id, x, y, z, radius):
        """添加球体障碍物"""
        co = CollisionObject()
        co.id = sphere_id
        co.header.frame_id = frame_id
        co.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [radius]

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        co.primitives = [primitive]
        co.primitive_poses = [pose.pose]

        self.psm.process_collision_object(co)

    def remove_object(self, object_id):
        """移除障碍物"""
        co = CollisionObject()
        co.id = object_id
        co.operation = CollisionObject.REMOVE
        self.psm.process_collision_object(co)
```

### 26.5.5 官方要点——碰撞检测官方模式与 PickNik Academy

MoveIt 官方 Collision Checking 页面说明默认后端是 FCL，并把碰撞对象分成三类：环境物体（collision object，unfixed）、附着物体（attached body，随末端执行器移动）与自碰撞（self-collision）——与本章 26.5 三类检测一一对应；合法的物体形状（box、sphere、cylinder、convex、mesh buffer）与"添加到 planning scene 后规划器自动避让"的行为也都是官方约定。PickNik Academy（MoveIt 官方培训机构，由 MoveIt 维护方 PickNik 运营）提供的 ROS 2 级别文档化培训直接对应本章四节内容，适合每一节结束后的官方同步练习，The Construct 与 Robotics Back-End 的课程则补充了从零构建配置包的完整视频演示。

## 26.6 启动MoveIt2

### 26.6.1 真实硬件启动

本仓库的 `xarm_ros2_arm_only` 仅配置仿真，不提供真实硬件驱动入口。真实硬件
需要使用设备厂商提供的 driver、控制器和 MoveIt 配置，并将规划组、关节名和
控制器映射迁移到对应工作区；不要直接使用下面的仿真命令控制实体机械臂。

### 26.6.2 仿真模式启动

```bash
# 课程提供的 xArm6 MoveIt + RViz（mock components，不启动 Gazebo）
ros2 launch xarm_ros2_arm_only arm_only_move_group.launch.py use_rviz:=true

# 课程提供的 Gazebo Harmonic + ros2_control + MoveIt2 仿真
ros2 launch xarm_ros2_arm_only arm_only.launch.py
```

### 26.6.3 启动文件解析

课程实际包名为 `xarm_ros2_arm_only`，其 `arm_only_move_group.launch.py` 使用
`MoveItConfigsBuilder` 加载 URDF/Xacro、SRDF、运动学、关节限位和 OMPL 配置，
然后启动 `move_group` 与可选的 RViz。`arm_only.launch.py` 在此基础上增加
Gazebo Harmonic、`gz_ros2_control`、控制器生成和机器人 spawn。配置文件位于
`src/xarm/config/`，启动文件位于 `src/xarm/launch/`。

## 课后练习

1. 使用MoveIt2 Setup Assistant为一个六自由度机械臂配置MoveIt2环境，生成完整的配置包。

2. 编写Python程序，使用MoveItPy API控制机械臂运动到多个关节目标位置，并记录规划时间。

3. 修改ompl_planning.yaml配置文件，设置不同的规划器（RRTConnect、PRM、RRT*），比较各规划器的规划成功率和路径质量。

4. 编写程序在规划场景中添加障碍物（盒体、球体、圆柱体），测试碰撞检测功能。

5. 解释MoveIt2的完整运动规划流程，从用户请求到轨迹执行的各个步骤。

---

## 仿真结合实例（当前仓库）：xArm6 的 OMPL 规划与碰撞检查

### 目标与知识点对应

使用课程提供的 MoveIt2 配置加载 URDF、SRDF、运动学和 OMPL 参数，在 RViz 中完成一次 xArm 关节空间规划，并在 Gazebo 模式下检查控制器连接。

### 运行步骤

先 source 外部兼容的 `xarm_description` 2.0.0，再从工作区根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source /path/to/xarm_description_workspace/install/setup.bash
ros2 launch xarm_ros2_arm_only arm_only_move_group.launch.py use_rviz:=true
```

在 RViz 的 MotionPlanning 面板选择 `xarm` 规划组，改变目标关节状态并点击 Plan。需要验证 Gazebo 控制链时执行：

```bash
ros2 launch xarm_ros2_arm_only arm_only.launch.py
ros2 control list_controllers
```

### 观察结果与边界

RViz 显示规划轨迹和碰撞模型；规划器、关节限位和规划组可在配置文件中核对。若缺少 xArm 描述或控制器，规划执行会失败；配置检查不等于真实硬件验证。

### 源码

相关源码包括 MoveIt 启动文件 `src/xarm/launch/arm_only_move_group.launch.py`、OMPL 配置 `src/xarm/config/arm_only_ompl_planning.yaml` 以及控制器配置 `src/xarm/config/arm_only_controllers.yaml`。

学习材料：
- MoveIt 2 官方文档 —— Concepts、MoveItPy 与 Collision Checking：https://moveit.picknik.ai/
- OMPL 官方文档 —— 规划器与 benchmark：https://ompl.kavrakilab.org/
- PickNik Academy —— MoveIt 官方培训课程：https://academy.picknik.ai/
- The Construct —— MoveIt 2 课程：https://www.theconstructsim.com/
- Robotics Back-End —— MoveIt 规划器对比教程：https://roboticsbackend.com/
