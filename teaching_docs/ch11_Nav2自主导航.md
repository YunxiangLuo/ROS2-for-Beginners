# 第11章 Nav2 自主导航

## 仿真结合实例（当前仓库）：Nav2 生命周期与目标导航

### 目标与知识点对应

使用预置地图、AMCL、规划器和控制器启动 Nav2，观察生命周期节点从配置到激活的过程，再发送 `NavigateToPose` 目标验证导航接口。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 终端 1：一键启动 Gazebo + Nav2，不启动 RViz 也可 headless 验证
ros2 launch navigation_sim_demo_ros2 nav2_demo.launch.py \
  use_gazebo:=true use_rviz:=true gz_headless:=false
```

```bash
# 终端 2：检查生命周期和发送目标
source install/setup.bash
ros2 run navigation_sim_demo_ros2 nav2_lifecycle_runner
ros2 run navigation_sim_demo_ros2 nav_goal_runner \
  --ros-args -p goal_x:=1.0 -p goal_y:=0.0
```

### 观察结果

- Nav2 启动日志应出现 `map_server`、`amcl`、`planner_server`、`controller_server` 等组件的配置/激活信息。
- RViz 可查看地图、机器人位姿、路径和代价地图；`nav_goal_runner` 在检测到里程计变化后输出 `navigation-motion-detected`。

### 源码与证据

- Launch：`src/navigation_sim_demo_ros2/launch/nav2_demo.launch.py`
- 生命周期：`src/navigation_sim_demo_ros2/navigation_sim_demo_ros2/nav2_lifecycle_runner.py`
- 目标：`src/navigation_sim_demo_ros2/navigation_sim_demo_ros2/nav_goal_runner.py`
- 参数/地图：`src/navigation_sim_demo_ros2/params/nav2_params.yaml`、`maps/Software_Museum.yaml`
- 启动日志证据：`lab_manuals/images/runtime/nonlab_nav2.png`

`nonlab_nav2.png` 主要证明导航组件加载与生命周期启动；完整到达目标的结果应在本地运行中按终端输出判断。

## 11.1 知识要点

### 11.1.1 Nav2 行为树架构

Nav2 使用行为树 (Behavior Tree) 作为任务编排的核心, 替代传统的有限状态机 (FSM)。

**行为树节点类型:**
```
Sequence (→): 顺序执行子节点, 任一失败则失败
Fallback (?): 尝试子节点, 任一成功则成功
ReactiveSequence: 顺序执行且可被更高优先级中断
Parallel (⇉): 并行执行多个子节点
Decorator: 装饰节点, 修改子节点行为
Action: 叶子节点, 执行具体任务 (导航/等待)
Condition: 叶子节点, 检查条件 (电池/障碍物)
```

**Nav2 默认行为树结构:**
```xml
<BehaviorTree>
  <PipelineSequence name="NavigateToPose">
    <RateController hz="10.0">
      <RecoveryNode>
        <ComputePathToPose goal="${goal}" planner_id="GridBased"/>
        <ClearEntireCostmap service_name="global_costmap/clear_entirely"/>
      </RecoveryNode>
    </RateController>
    <RateController hz="10.0">
      <RecoveryNode>
        <FollowPath controller_id="FollowPath"/>
        <Spin/>
        <Wait wait_duration="1.0"/>
        <ClearEntireCostmap service_name="local_costmap/clear_entirely"/>
      </RecoveryNode>
    </RateController>
  </PipelineSequence>
</BehaviorTree>
```

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap, ClearEntireCostmap
from geometry_msgs.msg import PoseStamped
import math


class Nav2Client(Node):
    """Nav2 Python 客户端 —— 使用 Action API"""
    def __init__(self):
        super().__init__('nav2_client')
        self.navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.clear_costmap_client = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_costmap'
        )
        self.get_logger().info('Nav2 客户端已就绪')

    def goto_pose(self, x: float, y: float, yaw: float = 0.0):
        """
        发送导航目标
        参数:
            x: 目标 x 坐标 (map 坐标系)
            y: 目标 y 坐标
            yaw: 目标朝向 (弧度)
        """
        if not self.navigate_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('导航服务器不可用')
            return None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        # yaw → quaternion
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.behavior_tree = ''  # 使用默认行为树

        future = self.navigate_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
        return future

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝')
            return
        self.get_logger().info('导航目标已接受')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'导航完成! 耗时: {result.total_elapsed_time.sec}s')
        else:
            self.get_logger().error(f'导航失败, 状态码: {status}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        remaining = feedback.distance_remaining
        self.get_logger().debug(f'剩余距离: {remaining:.2f} m')

    async def cancel_navigation(self):
        """取消当前导航任务"""
        if self.navigate_client.server_is_ready():
            self.navigate_client._cancel_goal_async()
```

### 11.1.2 全局规划器

全局规划器根据静态地图计算从当前位置到目标的最优路径。

| 规划器 | 算法 | 特点 | 适用场景 |
|--------|------|------|---------|
| NavFn | 导航函数/梯度下降 | 平滑路径, 经典算法 | 简单环境 |
| SmacPlannerHybrid | Hybrid-A* | Dubin 约束, 可反向 | 非全向机器人 |
| SmacPlannerLattice | 状态格 | 满足动力学约束 | 阿克曼转向 |
| ThetaStar | Theta* | 任意角度路径 | 全向机器人 |

```python
import numpy as np
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


def demonstrate_planners():
    """
    演示不同全局规划器切换
    NavigateToPose 的 behavior_tree 参数可以指定不同规划器
    """
    navigator = BasicNavigator()

    # 等待 Nav2 激活
    navigator.waitUntilNav2Active()

    # 目标位姿
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = 5.0
    goal.pose.position.y = 3.0
    goal.pose.orientation.w = 1.0

    # 规划器可以通过参数服务器动态切换 (不推荐运行时变)
    # 推荐在 launch 文件中配置
    # params.yaml:
    #   planner_server:
    #     ros__parameters:
    #       planner_plugin: "nav2_smac_planner/SmacPlannerHybrid"

    navigator.goToPose(goal)
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        print(f'剩余: {feedback.distance_remaining:.2f}m')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('导航成功!')
```

**自定义 Planner 接口 (Python):**
```python
# 通过继承 Planner 基类实现自定义全局规划器
import rclpy
from rclpy.node import Node
from nav2_core import GlobalPlanner
from nav_msgs.msg import Path


class SimpleStraightLinePlanner(Node, GlobalPlanner):
    """自定义直线全局规划器 (仅供教学)"""
    def __init__(self):
        super().__init__('straight_line_planner')

    def configure(self, parent_node, name, costmap, global_frame, ...):
        self.global_frame = global_frame
        self.costmap = costmap

    def createPlan(self, start: PoseStamped, goal: PoseStamped) -> Path:
        path = Path()
        path.header.frame_id = self.global_frame
        # 简单直线插值
        num_points = 50
        for i in range(num_points + 1):
            t = i / num_points
            pose = PoseStamped()
            pose.pose.position.x = start.pose.position.x * (1 - t) + goal.pose.position.x * t
            pose.pose.position.y = start.pose.position.y * (1 - t) + goal.pose.position.y * t
            pose.pose.orientation = start.pose.orientation
            path.poses.append(pose)
        return path
```

### 11.1.3 局部控制器

局部控制器负责执行全局规划的路径, 实现实时避障和速度控制。

| 控制器 | 算法 | 特点 | 适用场景 |
|--------|------|------|---------|
| DWB | 动态窗口逼近 | 评分机制选择最优速度 | 通用轮式机器人 |
| MPPI | 模型预测路径积分 | 采样+并行前向模拟 | 高性能动态环境 |
| RPP | 调节纯追踪 | 纯追踪+自适应前瞻 | 阿克曼/差分 |
| Regulated Pure Pursuit | RPP | 速度自适应缩放 | 简单可靠场景 |

```python
# DWB 控制器评分机制 (概念代码)
class DWBCritic:
    """DWB 控制器的核心 - 评分器 (Critic)"""
    def score_trajectory(self, trajectory):
        """
        对每条候选轨迹打分
        返回: 综合分数 (越低越好)
        """
        scores = {}
        # 1. 路径对齐评分
        scores['path_align'] = self.path_align_score(trajectory)
        # 2. 目标距离评分
        scores['goal_dist'] = self.goal_dist_score(trajectory)
        # 3. 障碍物距离评分 (安全评分)
        scores['obstacle'] = self.obstacle_cost_score(trajectory)
        # 4. 速度偏好评分
        scores['vel_pref'] = self.vel_pref_score(trajectory)
        # 5. 摆动评分 (抑制摇摆)
        scores['oscillation'] = self.oscillation_score(trajectory)
        # 加权求和
        return sum(scores.values())

    def obstacle_cost_score(self, trajectory):
        """障碍物代价值 - 越大越危险"""
        min_dist = min(trajectory.dist_to_obstacles)
        return 1.0 / (min_dist + 0.01) * 10.0


# 控制器参数调优示例 (nav2_params.yaml)
controller_params = {
    'FollowPath': {
        'plugin': 'dwb_core::DWBLocalPlanner',
        'debug_trajectory_details': True,
        'min_vel_x': 0.0,
        'max_vel_x': 0.5,
        'min_vel_y': 0.0,
        'max_vel_y': 0.0,        # 差速机器人, 无侧向运动
        'max_vel_theta': 1.0,
        'min_speed_xy': 0.0,
        'max_speed_xy': 0.5,
        'min_speed_theta': 0.0,
        'acc_lim_x': 2.5,
        'acc_lim_theta': 3.2,
        'decel_lim_x': -2.5,
        'decel_lim_theta': -3.2,
        'goal_checker_plugin': 'goal_checker',
        'critics': [
            'RotateToGoal',
            'Oscillation',
            'GoalAlign',
            'PathAlign',
            'PathDist',
            'GoalDist'
        ],
        # 各评分器权重
        'PathAlign.scale': 32.0,
        'GoalDist.scale': 24.0,
        'PathDist.scale': 32.0,
        'GoalAlign.scale': 24.0,
        'RotateToGoal.scale': 32.0,
        'Oscillation.scale': 10.0,
    }
}
```

### 11.1.4 代价地图分层配置

Costmap2D 采用分层架构, 每一层维护不同的代价信息:

```
┌────────────────────────────────────────┐
│         Master Costmap (融合层)        │
├────────────────────────────────────────┤
│  Static Layer   │ 障碍物地图             │
│  Obstacle Layer │ 传感器更新的动态障碍物  │
│  Inflation Layer│ 障碍物膨胀/安全距离     │
│  Range Layer    │ 距离传感器直接更新      │
│  Voxel Layer    │ 3D 体素感知            │
│  Keepout Layer  │ 禁区 (Filter Mask)     │
│  Speed Layer    │ 速度限制区域            │
└────────────────────────────────────────┘
```

```python
# 代价地图 YAML 配置示例 (以代码形式展示层次结构)
costmap_config = {
    'global_costmap': {
        'ros__parameters': {
            'update_frequency': 1.0,           # 全局代价地图更新频率低
            'publish_frequency': 1.0,
            'global_frame': 'map',
            'robot_base_frame': 'base_link',
            'robot_radius': 0.22,
            'resolution': 0.05,
            'track_unknown_space': True,
            'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
            'static_layer': {
                'plugin': 'nav2_costmap_2d::StaticLayer',
                'map_subscribe_transient_local': True,
            },
            'obstacle_layer': {
                'plugin': 'nav2_costmap_2d::ObstacleLayer',
                'enabled': True,
                'observation_sources': 'scan',
                'scan': {
                    'topic': '/scan',
                    'max_obstacle_height': 2.0,
                    'clearing': True,
                    'marking': True,
                    'data_type': 'LaserScan',
                },
            },
            'inflation_layer': {
                'plugin': 'nav2_costmap_2d::InflationLayer',
                'cost_scaling_factor': 3.0,
                'inflation_radius': 0.55,      # 膨胀半径
            },
        }
    },
    'local_costmap': {
        'ros__parameters': {
            'update_frequency': 5.0,            # 局部代价地图更新频率高
            'publish_frequency': 2.0,
            'global_frame': 'odom',
            'robot_base_frame': 'base_link',
            'rolling_window': True,             # 滚动窗口
            'width': 3,                         # 3m 窗口
            'height': 3,
            'resolution': 0.05,
            'robot_radius': 0.22,
            'plugins': ['voxel_layer', 'inflation_layer'],
            'voxel_layer': {
                'plugin': 'nav2_costmap_2d::VoxelLayer',
                'enabled': True,
                'publish_voxel_map': True,
                'origin_z': 0.0,
                'z_resolution': 0.05,
                'z_voxels': 16,
                'max_obstacle_height': 2.0,
                'mark_threshold': 0,
                'observation_sources': 'scan',
                'scan': {
                    'topic': '/scan',
                    'max_obstacle_height': 2.0,
                    'clearing': True,
                    'marking': True,
                    'data_type': 'LaserScan',
                },
            },
            'inflation_layer': {
                'plugin': 'nav2_costmap_2d::InflationLayer',
                'cost_scaling_factor': 3.0,
                'inflation_radius': 0.55,
            },
        }
    }
}
```

### 11.1.5 恢复行为

当机器人无法正常导航时, Nav2 会触发恢复行为:

```
NavigateToPose
  └── RecoveryNode (Fallback 风格)
       ├── ComputePathToPose (成功返回 SUCCESS)
       └── RecoverySequence
            ├── Spin (原地旋转 360°)
            ├── Wait (等待 1s)
            ├── BackUp (后退 0.3m)
            └── ClearCostmap (清除代价地图)
```

```python
from rclpy.node import Node
from nav2_core import Recovery

class CustomRecovery(Node, Recovery):
    """自定义恢复行为 —— 螺旋搜索"""
    def __init__(self):
        super().__init__('spiral_recovery')

    def configure(self, parent_node, name, tf_buffer, costmap_ros, global_costmap, local_costmap):
        self.parent_node = parent_node
        self.cmd_vel_pub = parent_node.create_publisher(Twist, '/cmd_vel', 10)

    def run(self):
        """执行螺旋搜索恢复"""
        self.get_logger().info('执行螺旋搜索恢复...')
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.5  # 逐渐扩大搜索半径
        self.cmd_vel_pub.publish(twist)
        rclpy.sleep(0.5)
        return True
```

### 11.1.6 Simple Commander API (Python)

Nav2 Simple Commander 是 Python 封装的高层 API, 简化导航调用:

```python
#!/usr/bin/env python3
"""
Nav2 Simple Commander 完整示例
参考: nav_goal_runner.py
"""
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math


class NavGoalRunner(Node):
    """
    多点导航任务执行器
    支持: goToPose, followWaypoints, cancelTask
    """
    def __init__(self):
        super().__init__('nav_goal_runner')
        self.navigator = BasicNavigator()
        self.get_logger().info('导航执行器已启动')

    def set_initial_pose(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        """设置初始位姿估计"""
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.068
        self.navigator.setInitialPose(pose)
        self.get_logger().info('初始位姿已设置')

    def wait_for_nav2(self):
        """等待 Nav2 激活"""
        self.get_logger().info('等待 Nav2 就绪...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 已就绪!')

    def create_pose(self, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
        """创建目标位姿消息"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def run_single_goal(self, x: float, y: float, yaw: float):
        """执行单点导航"""
        goal = self.create_pose(x, y, yaw)
        self.get_logger().info(f'导航至: ({x:.2f}, {y:.2f}, ∠{yaw:.2f})')
        self.navigator.goToPose(goal)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f'剩余距离: {feedback.distance_remaining:.2f}m, '
                    f'预计时间: {feedback.estimated_time_remaining:.1f}s'
                )

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('目标已到达!')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('导航已取消')
        elif result == TaskResult.FAILED:
            self.get_logger().error('导航失败')

    def run_waypoints(self, waypoints: list):
        """
        执行航点序列导航
        参数:
            waypoints: [(x1, y1, yaw1), (x2, y2, yaw2), ...]
        """
        poses = [self.create_pose(x, y, yaw) for x, y, yaw in waypoints]
        self.get_logger().info(f'开始航点导航: {len(poses)} 个目标')
        self.navigator.followWaypoints(poses)

        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 10 == 0:
                self.get_logger().info(
                    f'正在前往 #{feedback.current_waypoint}/{len(poses)}, '
                    f'剩余: {feedback.distance_remaining:.2f}m'
                )

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('所有航点已到达!')
        else:
            self.get_logger().error(f'航点导航中断: {result}')

    def patrol_loop(self, waypoints: list, loops: int = 3):
        """
        巡逻循环 —— 在航点间反复导航
        参数:
            waypoints: 巡逻点列表
            loops: 循环次数
        """
        for loop in range(loops):
            self.get_logger().info(f'--- 第 {loop + 1}/{loops} 轮巡逻 ---')
            for wp in waypoints:
                self.run_single_goal(*wp)
                if self.navigator.getResult() != TaskResult.SUCCEEDED:
                    return


def main():
    rclpy.init()
    runner = NavGoalRunner()

    # 设置初始位姿
    runner.set_initial_pose(0.0, 0.0, 0.0)
    runner.wait_for_nav2()

    # 示例: 单点导航
    runner.run_single_goal(3.0, 2.0, math.radians(90))

    # 示例: 航点导航
    waypoints = [
        (2.0, 0.0, 0.0),
        (3.0, 1.5, math.radians(90)),
        (2.0, 3.0, math.radians(180)),
        (0.0, 2.0, math.radians(-90)),
    ]
    runner.run_waypoints(waypoints)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 11.1.7 导航系统完整启动流程

```bash
# 1. 启动仿真环境
ros2 launch xbot_sim xbot_gazebo.launch.py world:=office

# 2. 启动 Nav2 Bringup (包含所有导航组件)
ros2 launch nav2_bringup navigation_launch.py \
  map:=src/maps/office_map.yaml \
  params_file:=config/nav2_params.yaml \
  use_sim_time:=true

# 3. (可选) 启动 SLAM (无地图时的 mapping 模式)
ros2 launch nav2_bringup slam_launch.py \
  params_file:=config/mapper_params_online_async.yaml
```

### 11.1.8 navigation_sim_demo 参考代码

```python
# 参考 ros2_ws/src/navigation_sim_demo_ros2/ 包
# 典型导航包结构:
navigation_sim_demo_ros2/
├── launch/
│   ├── nav2_demo.launch.py         # 主启动文件
│   └── navigation_custom.launch.py  # 自定义导航启动
├── config/
│   └── nav2_params.yaml             # Nav2 完整参数
├── nav_goal_runner.py               # Python 导航执行器
├── patrol_demo.py                   # 巡逻演示
└── dynamic_obstacle_spawner.py      # 动态障碍物生成
```

---

## 11.2 练习题

**1. 架构题:** 绘制 Nav2 行为树结构图, 说明 RecoveryNode 的工作机制,并解释为什么使用 Fallback 节点作为恢复树根节点。

**2. 配置题:** 为差速轮式机器人配置 DWB 控制器参数: 最大线速度 0.5 m/s, 最大角速度 1.0 rad/s, 膨胀半径 0.55m, 并写出来调节 PathAlign 和 GoalDist 权重对导航行为的影响。

**3. 编程题:** 使用 nav2_simple_commander 编写一个 "8 字形" 巡逻任务: 机器人轮流经过 4 个目标点, 每次到达后原地旋转 360°, 共执行 3 轮。

**4. 分析题:** 比较全局代价地图和局部代价地图在更新频率、坐标系、大小和插件配置上的差异, 说明为什么需要两种代价地图。

**5. 设计题:** 设计一个自定义恢复行为: "后退并重新规划" — 当导航失败时, 机器人先后退 0.5m 再尝试重新规划路径。写出 XML 行为树片段和 Python 实现框架。

**6. 综合题:** 某仓库场景下机器人需要在狭窄通道中导航, 经常出现路径规划失败。设计方案改进包括: 代价地图分辨率调整、规划器替换为 Hybrid-A*、增加 Keepout Filter 避免特定区域, 并说明各参数调整的预期效果。
