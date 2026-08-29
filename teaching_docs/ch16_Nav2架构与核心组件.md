# 第16章 Nav2架构与核心组件

## 仿真结合实例（当前仓库）：查看 Nav2 组件生命周期

### 目标与知识点对应

用 `navigation_sim_demo_ros2` 在 Gazebo Wheeltec 场景中启动 Nav2，观察 `map_server`、`amcl`、`planner_server`、`controller_server` 和 `bt_navigator` 的生命周期编排，对应本章的组件分层与行为树入口。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch navigation_sim_demo_ros2 nav2_demo.launch.py \
  use_gazebo:=true use_rviz:=true gz_headless:=false
```

```bash
ros2 run navigation_sim_demo_ros2 nav2_lifecycle_runner
ros2 node list
ros2 topic info /map
ros2 action list | grep navigate
```

### 观察结果

终端可看到定位组和导航组依次配置、激活；RViz 可查看地图、代价地图、TF 和导航相关显示。`nav2_params.yaml` 中的插件配置决定规划器、控制器和代价地图层。

### 源码与证据

- Launch：`src/navigation_sim_demo_ros2/launch/nav2_demo.launch.py`
- 生命周期：`src/navigation_sim_demo_ros2/navigation_sim_demo_ros2/nav2_lifecycle_runner.py`
- 参数：`src/navigation_sim_demo_ros2/params/nav2_params.yaml`
- 终端证据：`lab_manuals/images/runtime/nonlab_nav2.png`

证据主要证明组件启动；目标到达和控制效果需在本地运行中单独检查。

## 学习目标
- 理解Navigation2整体架构和设计思想
- 掌握行为树驱动的导航决策框架
- 熟悉ROS2生命周期节点管理机制
- 了解Nav2核心组件及其功能
- 能够配置和启动Nav2导航系统

## 16.1 机器人导航概述

### 16.1.1 导航的三大问题

机器人自主导航需要解决三个核心问题：

**1. 我在哪里？ — 定位问题**
- 使用AMCL或SLAM确定机器人在地图中的位置
- 输入：激光雷达、里程计、地图
- 输出：机器人位姿估计

**2. 我要去哪里？ — 目标设定**
- 用户或任务系统指定目标位姿
- 可通过RViz2可视化设定
- 或通过上层调度系统下发目标点

**3. 我要怎么去？ — 规划与控制**
- 全局规划：计算从起点到终点的无碰撞路径
- 局部规划：实时避障和路径跟踪
- 运动控制：生成速度指令驱动机器人

### 16.1.2 Nav2概述

Nav2 (Navigation2) 是ROS2的官方导航框架，替代ROS1中的move_base。它采用行为树驱动的插件化架构，支持高度可定制的导航行为。

**Nav2的设计目标：**
- 模块化：核心功能以插件形式提供
- 可扩展：支持自定义规划器、控制器、恢复行为
- 鲁棒性：行为树驱动的容错机制
- 生命周期管理：资源高效利用

## 16.2 Nav2整体架构

### 16.2.1 系统架构图

Nav2的架构可以分解为四个层次：

```
用户接口层:
┌──────────────────────────────────────┐
│ RViz2 │ Simple Commander │ 任务调度 │
└──────────────────────────────────────┘
         ↓ 导航目标 (NavigateToPose Action)
┌──────────────────────────────────────┐
│    行为树导航器 (BT Navigator)       │
│   - 行为树执行引擎                    │
│   - 恢复行为管理                      │
└──────────────────────────────────────┘
         ↓                  ↓
┌──────────────────┐ ┌──────────────────┐
│  Planner Server  │ │Controller Server │
│  全局路径规划     │ │ 局部路径跟踪     │
│  Dijkstra/A*     │ │ DWA/RPP/MPPI     │
└──────────────────┘ └──────────────────┘
         ↓                  ↓
┌──────────────────┐ ┌──────────────────┐
│ Global Costmap   │ │ Local Costmap    │
│ 静态层/障碍物层   │ │ 障碍物层/膨胀层  │
│ 膨胀层            │ │ 滚动窗口         │
└──────────────────┘ └──────────────────┘
         ↓                  ↓
┌──────────────────────────────────────┐
│      传感器/里程计/地图/定位          │
│   LaserScan, Odometry, Map, AMCL     │
└──────────────────────────────────────┘
```

### 16.2.2 核心组件

Nav2包含以下核心组件：

| 组件 | 功能 | 对应ROS1 |
|------|------|----------|
| `bt_navigator` | 行为树导航决策引擎 | move_base |
| `planner_server` | 全局路径规划 | global_planner |
| `controller_server` | 局部路径规划与控制 | local_planner |
| `costmap_2d` | 全局/局部代价地图 | costmap_2d |
| `behavior_server` | 恢复行为管理 | recovery_behaviors |
| `amcl` | 粒子滤波定位 | amcl |
| `map_server` | 地图加载与服务 | map_server |
| `waypoint_follower` | 航点跟随 | - |

### 16.2.3 Nav2 vs move_base

| 特性 | Nav2 (ROS2) | move_base (ROS1) |
|------|------------|-----------------|
| 架构 | 行为树驱动 | 有限状态机 |
| 可扩展性 | 插件化，高度可定制 | 有限的自定义 |
| 生命周期 | ROS2生命周期节点 | 标准节点 |
| 多机器人 | 原生支持 | 需额外配置 |
| 恢复行为 | 可配置行为树 | 固定恢复序列 |
| 语言 | C++/Python | C++ |

## 16.3 行为树驱动导航

### 16.3.1 行为树基础

行为树 (Behavior Tree, BT) 是一种用于控制决策流程的树形结构，具有以下特点：

**核心节点类型：**

```
控制节点:
Sequence (→) : 顺序执行子节点，所有成功才成功
Fallback (?) : 尝试子节点，任一成功则成功
Parallel (⇉) : 并行执行子节点

装饰节点:
Repeater     : 重复执行子节点
Inverter     : 反转子节点结果
RateController: 控制执行频率
Condition    : 条件检查

执行节点:
Action  : 执行具体任务（导航、旋转等）
Condition: 检查条件（是否到达目标等）
```

**行为树与传统FSM的对比：**

| 特性 | 行为树 | 有限状态机 |
|------|--------|-----------|
| 模块化 | 高，节点可重用 | 低，状态转换耦合 |
| 可读性 | 树形结构直观 | 状态图复杂 |
| 调试 | 可视化XML | 日志跟踪 |
| 容错 | 自然支持 | 需显式设计 |
| 复用 | 节点可跨树重用 | 状态难复用 |

### 16.3.2 Nav2默认行为树

Nav2的默认导航行为树 `navigate_to_pose.xml`：

```xml
<!-- navigation_nav2/behavior_trees/navigate_to_pose.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithRecovery">
      <!-- 全局规划阶段 -->
      <RateController hz="1.0">
        <RecoveryNode number_of_retries="1" name="ComputePath">
          <ComputePathToPose 
            goal="{goal}" 
            path="{path}" 
            planner_id="GridBased"/>
          <ClearEntireCostmap 
            service_name="global_costmap/clear_entirely"/>
        </RecoveryNode>
      </RateController>
      
      <!-- 路径跟踪阶段 -->
      <RateController hz="10.0">
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath 
            path="{path}" 
            controller_id="FollowPath"/>
          <Sequence name="RecoveryActions">
            <Spin 
              spin_dist="1.57"/>
            <Wait 
              wait_duration="1.0"/>
            <BackUp 
              backup_dist="0.15"/>
            <ClearEntireCostmap 
              service_name="local_costmap/clear_entirely"/>
          </Sequence>
        </RecoveryNode>
      </RateController>
    </PipelineSequence>
  </BehaviorTree>
</root>
```

### 16.3.3 行为树的工作原理

```
行为树执行流程:
1. 接收到导航目标 {goal}
2. 进入 PipelineSequence
3. 第一阶段: RateController(1Hz)
   ├── ComputePathToPose: 计算全局路径
   │   ├── 成功 → 进入第二阶段
   │   └── 失败 → RecoveryNode
   │       ├── ClearEntireCostmap: 清除代价地图
   │       └── 重试 ComputePathToPose
   └── 继续第一阶段

4. 第二阶段: RateController(10Hz)
   ├── FollowPath: 执行路径跟踪
   │   ├── 成功 → 导航完成
   │   └── 失败 → RecoveryNode
   │       ├── Spin: 原地旋转
   │       ├── Wait: 等待
   │       ├── BackUp: 后退
   │       ├── ClearEntireCostmap: 清除代价地图
   │       └── 重试 FollowPath
   └── 继续第二阶段
```

### 16.3.4 自定义行为树

用户可以根据需求创建自定义行为树：

```xml
<!-- custom_navigation.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="CustomNavigate">
      <!-- 步骤1: 检查电池电量 -->
      <BatteryCheck 
        min_battery_level="0.2"
        battery_topic="/battery_status"/>
      
      <!-- 步骤2: 计算路径 -->
      <ComputePathToPose 
        goal="{goal}" 
        path="{path}"/>
      
      <!-- 步骤3: 使用MPPI控制器跟踪路径 -->
      <FollowPath 
        path="{path}" 
        controller_id="MPPI"/>
      
      <!-- 步骤4: 到达后执行额外动作 -->
      <Sequence name="PostArrival">
        <Wait wait_duration="2.0"/>
        <Spin spin_dist="6.2832"/>
      </Sequence>
    </Sequence>
  </BehaviorTree>
</root>
```

## 16.4 生命周期节点管理

### 16.4.1 生命周期节点概念

Nav2使用ROS2的**生命周期节点 (Lifecycle Node)** 管理所有导航组件。生命周期节点具有明确的状态转换过程：

```
                    │
                    ↓
             ┌─────────────┐
             │ Unconfigured│
             └─────────────┘
                    │ configure()
                    ↓
             ┌─────────────┐
             │  Inactive   │ ←── 配置完成，未激活
             └─────────────┘
                    │ activate()
                    ↓
             ┌─────────────┐
             │   Active    │ ←── 正常运行
             └─────────────┘
                    │ deactivate()
                    ↓
             ┌─────────────┐
             │  Inactive   │
             └─────────────┘
                    │ cleanup()
                    ↓
             ┌─────────────┐
             │ Unconfigured│ ←── 回到初始状态
             └─────────────┘
                    │ shutdown()
                    ↓
             ┌─────────────┐
             │ Finalized   │ ←── 节点销毁
             └─────────────┘
```

**状态说明：**
- **Unconfigured:** 初始状态，节点已创建但未初始化
- **Inactive:** 配置完成，可以发布/订阅但不处理数据
- **Active:** 正常运行，执行核心功能
- **Finalized:** 节点已销毁

### 16.4.2 生命周期管理服务

```bash
# 查看当前生命周期状态
ros2 service call /bt_navigator/get_state \
  lifecycle_msgs/srv/GetState

# 配置节点 (Unconfigured → Inactive)
ros2 service call /bt_navigator/change_state \
  lifecycle_msgs/srv/ChangeState \
  "{transition: {id: 1}}"   # 1=TRANSITION_CONFIGURE

# 激活节点 (Inactive → Active)
ros2 service call /bt_navigator/change_state \
  lifecycle_msgs/srv/ChangeState \
  "{transition: {id: 3}}"   # 3=TRANSITION_ACTIVATE

# 停用节点 (Active → Inactive)
ros2 service call /bt_navigator/change_state \
  lifecycle_msgs/srv/ChangeState \
  "{transition: {id: 4}}"   # 4=TRANSITION_DEACTIVATE

# 清理节点 (Inactive → Unconfigured)
ros2 service call /bt_navigator/change_state \
  lifecycle_msgs/srv/ChangeState \
  "{transition: {id: 2}}"   # 2=TRANSITION_CLEANUP
```

### 16.4.3 生命周期管理器

Nav2使用 `nav2_lifecycle_manager` 统一管理所有导航组件的生命周期：

```yaml
# nav2_params.yaml
lifecycle_manager:
  lifecycle_manager:
    ros__parameters:
      autostart: true                  # 自动启动
      node_names:                      # 管理的节点列表
        - bt_navigator
        - planner_server
        - controller_server
        - behavior_server
        - local_costmap
        - global_costmap
        - amcl
        - map_server
```

```bash
# 使用生命周期管理器一键启动
ros2 launch nav2_bringup navigation_launch.py \
  map:=./maps/my_map.yaml \
  use_sim_time:=true

# 生命周期管理器自动执行:
# 1. 创建所有节点 → Unconfigured
# 2. 配置所有节点 → Inactive
# 3. 激活所有节点 → Active
```

### 16.4.4 自定义生命周期节点

```python
import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

class Nav2Component(LifecycleNode):
    """自定义Nav2生命周期组件"""
    def __init__(self):
        super().__init__('custom_nav_component')
        
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """配置阶段：分配资源、读取参数"""
        self.get_logger().info('正在配置...')
        
        # 读取参数
        self.declare_parameter('param1', 'default')
        self.param1 = self.get_parameter('param1').value
        
        # 创建发布器和订阅器（但不激活）
        self.publisher = self.create_lifecycle_publisher(
            PoseStamped, '/custom_pose', 10)
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.get_logger().info('配置完成')
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """激活阶段：开始处理数据"""
        self.get_logger().info('正在激活...')
        
        # 激活发布器
        self.publisher.on_activate()
        
        self.get_logger().info('已激活')
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """停用阶段：暂停处理"""
        self.get_logger().info('正在停用...')
        self.publisher.on_deactivate()
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """清理阶段：释放资源"""
        self.get_logger().info('正在清理...')
        self.publisher = None
        self.subscriber = None
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """关闭阶段"""
        self.get_logger().info('正在关闭...')
        return TransitionCallbackReturn.SUCCESS
    
    def scan_callback(self, msg):
        """激光数据回调（仅在Active状态处理）"""
        pass
```

## 16.5 核心组件详解

### 16.5.1 BT Navigator

BT Navigator是Nav2的行为树执行引擎，负责：

- 加载和执行行为树XML
- 管理导航任务的生命周期
- 提供NavigateToPose和FollowWaypoints的Action接口

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math

class BTNavigatorClient(Node):
    """BT Navigator Action客户端"""
    def __init__(self):
        super().__init__('bt_navigator_client')
        
        self.action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
            
    def send_goal(self, x: float, y: float, yaw: float,
                  behavior_tree: str = ''):
        """发送导航目标"""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        
        if behavior_tree:
            goal.behavior_tree = behavior_tree
            
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(
            self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            return
        self.get_logger().info('目标已接受')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
        
    def result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'导航成功!')
        else:
            self.get_logger().error(f'导航失败, status={status}')
            
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if feedback.distance_remaining > 0:
            self.get_logger().info(
                f'剩余距离: {feedback.distance_remaining:.2f}m',
                throttle_duration_sec=1.0)
```

### 16.5.2 Planner Server

全局规划器负责在代价地图上搜索从起点到终点的无碰撞路径：

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class GlobalPlannerClient(Node):
    """全局规划器客户端"""
    def __init__(self):
        super().__init__('global_planner_client')
        
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.planner_action_client = ActionClient(
            self, ComputePathToPose, '/compute_path_to_pose')
            
    def request_plan(self, start: PoseStamped, goal: PoseStamped,
                      planner_id: str = 'GridBased'):
        """请求全局路径规划"""
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = goal
        goal_msg.planner_id = planner_id
        
        self.planner_action_client.wait_for_server()
        future = self.planner_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.plan_response_callback)
        
    def plan_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('规划请求被拒绝')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.plan_result_callback)
        
    def plan_result_callback(self, future):
        result = future.result().result
        if result.path.poses:
            self.get_logger().info(
                f'路径规划成功: {len(result.path.poses)}个路径点')
```

### 16.5.3 Controller Server

局部控制器负责跟踪全局路径并避开动态障碍物：

- 支持多种控制器插件（DWB, MPPI, Regulated Pure Pursuit）
- 实时处理激光传感器数据
- 输出 `cmd_vel` 速度指令

### 16.5.4 Behavior Server

恢复行为服务器管理导航失败时的恢复策略：

- Spin：原地旋转
- BackUp：后退
- Drive on Heading：沿当前方向前进
- Wait：等待
- Clear Costmap：清除代价地图

### 16.5.5 Costmap Layers

代价地图层负责将环境建模为带代价值的栅格：

| 图层 | 插件 | 功能 |
|------|------|------|
| 静态层 | StaticLayer | 加载预建地图 |
| 障碍物层 | ObstacleLayer | 传感器实时障碍物 |
| 膨胀层 | InflationLayer | 障碍物膨胀 |
| 体素层 | VoxelLayer | 3D障碍物感知 |

## 16.6 Nav2启动与配置

### 16.6.1 启动文件

```bash
# 使用预建地图启动导航
ros2 launch nav2_bringup navigation_launch.py \
  map:=./maps/office_map.yaml \
  params_file:=./config/nav2_params.yaml \
  use_sim_time:=true

# 自动启动（包含定位）
ros2 launch nav2_bringup bringup_launch.py \
  map:=./maps/office_map.yaml \
  use_sim_time:=true

# 仅定位模式
ros2 launch nav2_bringup localization_launch.py \
  map:=./maps/office_map.yaml \
  use_sim_time:=true
```

### 16.6.2 参数文件结构

```yaml
# nav2_params.yaml 整体结构
bt_navigator:
  ros__parameters:
    ...  # 行为树配置

planner_server:
  ros__parameters:
    ...  # 规划器配置

controller_server:
  ros__parameters:
    ...  # 控制器配置

local_costmap:
  local_costmap:
    ros__parameters:
      ...  # 局部代价地图

global_costmap:
  global_costmap:
    ros__parameters:
      ...  # 全局代价地图

amcl:
  ros__parameters:
    ...  # AMCL定位配置

map_server:
  ros__parameters:
    ...  # 地图服务配置

behavior_server:
  ros__parameters:
    ...  # 恢复行为配置
```

### 16.6.3 Simple Commander API

Nav2提供了Python的Simple Commander API，简化导航调用：

```python
#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import math

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    # 等待Nav2就绪
    navigator.waitUntilNav2Active()
    
    # 创建目标位姿
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = 3.0
    goal.pose.position.y = 2.0
    goal.pose.orientation.w = 1.0
    
    # 发送导航目标
    navigator.goToPose(goal)
    
    # 监控执行过程
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f'剩余距离: {feedback.distance_remaining:.2f}m')
    
    # 检查结果
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('导航成功!')
    else:
        print(f'导航失败: {result}')
    
    rclpy.shutdown()
```

## 课后练习

1. **架构题:** 绘制Nav2的系统架构图，说明各组件之间的数据流和依赖关系。

2. **配置题:** 编写Nav2的nav2_params.yaml配置文件，包括行为树导航器、全局规划器、局部控制器和代价地图的完整配置。

3. **编程题:** 使用nav2_simple_commander编写一个多点导航程序，机器人依次访问5个目标点，每到达一个点后暂停3秒钟。

4. **分析题:** 分析Nav2使用生命周期节点管理的优势，说明生命周期状态转换机制如何提高系统的鲁棒性和资源利用效率。

5. **设计题:** 设计一个自定义的导航行为树，包含以下功能：电池检查、前往充电桩、充电等待、继续任务。写出对应的XML行为树文件。
