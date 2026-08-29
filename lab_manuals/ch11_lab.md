# 第11章 实验手册: Nav2 自主导航

## 当前仓库仿真验证：Nav2 生命周期与导航目标

### 实验目标

在当前 Wheeltec Gazebo 场景中启动 Nav2，检查定位组、规划组和控制组的生命周期，并发送一个 `NavigateToPose` 目标。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 终端 1：Gazebo + Nav2 + RViz
ros2 launch navigation_sim_demo_ros2 nav2_demo.launch.py \
  use_gazebo:=true use_rviz:=true gz_headless:=false
```

```bash
# 终端 2：生命周期检查
source install/setup.bash
ros2 run navigation_sim_demo_ros2 nav2_lifecycle_runner
```

```bash
# 终端 3：发送目标并监测运动
source install/setup.bash
ros2 run navigation_sim_demo_ros2 nav_goal_runner \
  --ros-args -p goal_x:=1.0 -p goal_y:=0.0
```

### 观察与验收

启动日志应出现 `map_server`、`amcl`、`planner_server`、`controller_server` 和 `bt_navigator`；RViz 可查看地图、路径和代价地图。组件启动证据：`images/runtime/nonlab_nav2.png`；目标是否到达以本地 `/odom` 和程序输出为准。

## 环境说明

本实验使用 XBot-U 机器人 + Gazebo Sim (Fortress) 仿真, 预建地图位于 `/home/yun/maps/`:

| 地图文件 | 尺寸 | 分辨率 | 原点 |
|----------|------|--------|------|
| `lab10_e2e_replayed_map.yaml` | 202×202 | 0.05 m/pix | `[-5.05, -5.04, 0]` |
| `lab10_map.yaml` | 198×200 | 0.05 m/pix | `[-4.89, -5, 0]` |

**控制链:**
```
/cmd_vel_teleop (teleop) ─┐
                           ├── /twist_mux → /cmd_vel → GazeboInterfaceBridge
/cmd_vel_nav (Nav2) ──────┘                      → /wheel_velocity_controller/commands
```

## 准备工作

### 构建包
```bash
cd ~/ros2_course_ws
colcon build --packages-select navigation_lab
source install/setup.bash
```

### 启动仿真 + Nav2

```bash
# 终端1: Gazebo 仿真 
ros2 launch robot_sim_demo_ros2 sim_bringup.launch.py \
  fake_laser_map_yaml:=/home/yun/maps/lab10_e2e_replayed_map.yaml

# 终端2: Nav2 导航栈
ros2 launch navigation_lab nav_bringup.launch.py \
  map:=/home/yun/maps/lab10_e2e_replayed_map.yaml \
  use_composition:=False autostart:=true

# 终端3: RViz2 可视化 (Nav2 默认视图, 含规划路径/代价地图)
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

## 练习1: goToPose 单点导航 (~15 分钟)

### 目标
使用 Nav2 Simple Commander API 实现单目标点自主导航, 理解 `NavigateToPose` action 的反馈结构。

### 知识点

`NavigateToPose` 的 action 反馈包含以下字段:

| 字段 | 类型 | 说明 |
|------|------|------|
| `distance_remaining` | float32 | 到目标的剩余距离 (米) |
| `navigation_time` | Duration | 已导航时间 |
| `estimated_time_remaining` | Duration | 预计剩余时间 |

### 代码

源码位置: `navigation_lab/navigation_lab/go_to_pose_demo.py`

```python
import math

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from .nav_utils import make_pose_stamped


def main():
    rclpy.init()
    nav = BasicNavigator()

    nav.setInitialPose(make_pose_stamped('map', 0.0, 0.0, 0.0))
    nav.waitUntilNav2Active()

    goals = [
        (3.0, 0.0, 0.0),
        (3.0, 2.0, math.radians(90)),
        (0.0, 2.0, math.radians(180)),
        (0.0, 0.0, math.radians(-90)),
    ]

    for i, (x, y, yaw) in enumerate(goals):
        nav.get_logger().info(f'--- Goal {i+1}/{len(goals)} ---')
        goal = make_pose_stamped('map', x, y, yaw)
        nav.goToPose(goal)

        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                nav.get_logger().info(
                    f'  distance_remaining={feedback.distance_remaining:.2f}m, '
                    f'estimated_time={feedback.estimated_time_remaining.sec}s'
                )

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            nav.get_logger().info(f'Goal {i+1} reached!')
        elif result == TaskResult.CANCELED:
            nav.get_logger().warn(f'Goal {i+1} canceled')
        elif result == TaskResult.FAILED:
            nav.get_logger().error(f'Goal {i+1} failed')
        else:
            nav.get_logger().error(f'Goal {i+1} unknown result')

    nav.get_logger().info('All goals completed!')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 运行
```bash
# 确认仿真和 Nav2 已启动后
ros2 run navigation_lab go_to_pose_demo
```

### 观察
- 终端输出: `distance_remaining` 逐渐减小, `estimated_time_remaining` 动态更新
- RViz2 (nav2_default_view.rviz) 已预置以下显示:
  - **Map** (`/map`) — 静态地图
  - **Global Costmap** (`/global_costmap/costmap`) — 全局代价地图
  - **Local Costmap** (`/local_costmap/costmap`) — 局部代价地图
  - **Path** (`/plan`) — 全局规划路径 (绿色)
  - **Local Plan** (`/local_plan`) — 局部规划轨迹 (蓝色)
  - **Amcl Particle Swarm** (`/particle_cloud`) — AMCL 粒子云
  - **Navigation 2 面板** — 提供 "2D Pose Estimate" 和 "2D Nav Goal" 工具
- 如果 RViz2 中未自动显示数据, 检查 `Fixed Frame` 是否为 `map`, 以及 Nav2 是否已正常启动

### 验收标准
- [ ] 机器人依次到达 4 个目标点
- [ ] 终端正确显示 `distance_remaining` 和 `estimated_time_remaining`
- [ ] 理解 `goToPose` 是异步调用, 需轮询 `isTaskComplete()` + `getFeedback()`

---

## 练习2: followWaypoints 多点导航 (~15 分钟)

### 目标
掌握 `FollowWaypoints` action 的航点序列导航, 理解其反馈机制 (与 `NavigateToPose` 不同)。

### 知识点
- `FollowWaypoints` 的反馈仅包含 `current_waypoint` (uint32) — **没有** `distance_remaining`
- 机器人依次导航到序列中的每个航点
- 默认 `waypoint_follower` 到达每个航点后停留 200ms

### 代码

源码位置: `navigation_lab/navigation_lab/follow_waypoints_demo.py`

```python
import math

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from .nav_utils import make_pose_stamped


def main():
    rclpy.init()
    nav = BasicNavigator()

    nav.setInitialPose(make_pose_stamped('map', 0.0, 0.0, 0.0))
    nav.waitUntilNav2Active()

    waypoints = [
        make_pose_stamped('map', 2.0, 0.0, 0.0),
        make_pose_stamped('map', 2.0, 2.0, math.radians(90)),
        make_pose_stamped('map', 0.0, 2.0, math.radians(180)),
        make_pose_stamped('map', 0.0, 0.0, math.radians(-90)),
    ]

    nav.followWaypoints(waypoints)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            nav.get_logger().info(
                f'current_waypoint={feedback.current_waypoint}/{len(waypoints)}'
            )

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.get_logger().info('All waypoints reached!')
    elif result == TaskResult.CANCELED:
        nav.get_logger().warn('Waypoint task canceled')
    elif result == TaskResult.FAILED:
        nav.get_logger().error('Waypoint task failed')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 运行
```bash
ros2 run navigation_lab follow_waypoints_demo
```

### 观察
- 终端输出 `current_waypoint` 递增: `1/4` → `2/4` → `3/4` → `4/4` → SUCCEEDED
- RViz2 中观察机器人按顺序通过所有航点

### 验收标准
- [ ] 机器人按顺序访问所有 4 个航点
- [ ] 终端正确显示 `current_waypoint` 序号
- [ ] 将 `stop_on_failure` 设为 `true` 再测试 (修改 `config/navigation_lab.yaml` 后重启 Nav2)

---

## 练习3: 恢复行为与动态障碍 (~20 分钟)

### 目标
了解 Nav2 的恢复行为 (Spin, BackUp, ClearCostmap) 触发条件, 并通过合成 LaserScan 模拟动态障碍物。

### 3.1 恢复行为 Demo

源码位置: `navigation_lab/navigation_lab/recovery_demo.py`

```python
import math

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from .nav_utils import make_pose_stamped


def main():
    rclpy.init()
    nav = BasicNavigator()

    nav.setInitialPose(make_pose_stamped('map', 0.0, 0.0, 0.0))
    nav.waitUntilNav2Active()

    nav.get_logger().info('Navigating toward obstacles to trigger recovery...')
    goal = make_pose_stamped('map', 4.0, 0.0, 0.0)
    nav.goToPose(goal)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            nav.get_logger().info(
                f'distance_remaining={feedback.distance_remaining:.2f}m, '
                f'navigation_time={feedback.navigation_time.sec}s, '
                f'estimated_time_remaining={feedback.estimated_time_remaining.sec}s'
            )

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.get_logger().info('Goal reached!')
    elif result == TaskResult.CANCELED:
        nav.get_logger().warn('Canceled')
    elif result == TaskResult.FAILED:
        nav.get_logger().error('Failed — recovery behaviors were triggered')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 运行
```bash
ros2 run navigation_lab recovery_demo
```

#### 观察
- 当机器人在障碍物前卡住时, 自动触发 Spin (原地旋转) 或 BackUp (后退)
- 终端输出 `number_of_recoveries` 字段 (在 `NavigateToPose` feedback 中)
- RViz2 中观察代价地图清除和重新规划

### 3.2 合成激光扫描注入 (模拟动态障碍物)

由于 Gazebo Sim 的 `fake_laser` 基于预计算 2D 占用, 无法响应新生成的实体, 故使用合成 LaserScan 消息模拟动态障碍物。

源码位置: `navigation_lab/navigation_lab/scan_injector.py`

```python
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanInjector(Node):
    def __init__(self):
        super().__init__('scan_injector')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)
        self.angle = 0.0

    def publish_scan(self):
        self.angle += 0.02
        scan = LaserScan()
        scan.header.frame_id = 'laser'
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0
        scan.time_increment = 1.0 / 360.0 / 180.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 3.5

        obstacle_x = 2.0 + 0.8 * math.sin(self.angle * 0.5)
        obstacle_y = 0.0 + 0.8 * math.cos(self.angle * 0.5)

        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        ranges = []
        for i in range(num_readings):
            theta = scan.angle_min + i * scan.angle_increment
            dx = obstacle_x - 0.0
            dy = obstacle_y - 0.0
            dist = math.sqrt(dx * dx + dy * dy)
            angle_to_obs = math.atan2(dy, dx)
            angle_diff = abs(theta - angle_to_obs)
            if angle_diff < 0.05 and dist < 3.0 and dist > 0.1:
                ranges.append(dist)
            else:
                ranges.append(3.5)
        scan.ranges = ranges
        scan.intensities = [0.0] * num_readings
        self.pub.publish(scan)


def main():
    rclpy.init()
    node = ScanInjector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 运行
```bash
# 终端4: 启动 scan 注入器
ros2 run navigation_lab scan_injector
```

#### 观察
- 合成障碍物以圆形轨迹在 (2.0, 0.0) 附近移动
- RViz2: 局部代价地图上观察到移动的障碍物
- 机器人尝试绕过障碍物

### 验收标准
- [ ] 恢复 demo 显示 `distance_remaining`, `navigation_time`, `estimated_time_remaining` 全部正确
- [ ] 机器人卡住时触发 Spin/BackUp 行为
- [ ] scan_injector 发布的消息被 Nav2 接收, 代价地图中显示动态障碍物

---

## 练习4: 导航过程监控 (~15 分钟)

### 目标
编写一个监控节点, 实时监听导航过程中的关键话题, 理解 Nav2 各模块间的数据流。

### 代码

源码位置: `navigation_lab/navigation_lab/nav_monitor.py`

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from nav2_msgs.msg import Costmap


class NavMonitor(Node):
    def __init__(self):
        super().__init__('nav_monitor')
        self.create_subscription(Path, '/plan', self.plan_cb, 10)
        self.create_subscription(Path, '/local_plan', self.local_plan_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.plan_count = 0
        self.last_plan_key = None

    def plan_cb(self, msg):
        if not msg.poses:
            return
        key = (msg.poses[0].pose.position.x, msg.poses[0].pose.position.y,
               msg.poses[-1].pose.position.x, msg.poses[-1].pose.position.y)
        if key != self.last_plan_key:
            self.plan_count += 1
            self.get_logger().info(f'New global plan #{self.plan_count}')
            self.last_plan_key = key

    def local_plan_cb(self, msg):
        self.get_logger().info(f'Local plan: {len(msg.poses)} poses')

    def cmd_cb(self, msg):
        self.get_logger().info(
            f'Cmd vel: linear={msg.linear.x:.2f} angular={msg.angular.z:.2f}'
        )

    def odom_cb(self, msg):
        pass


def main():
    rclpy.init()
    node = NavMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 运行
```bash
# 与任一导航 demo 同时运行
ros2 run navigation_lab nav_monitor
```

### 观察
- 全局规划重规划次数 — 障碍物触发新规划时计数递增
- `/local_plan` 的轨迹长度变化 — 反映避障机动
- `/cmd_vel` 速度指令 — 观察减速转向行为

### 验收标准
- [ ] 终端显示全局路径重规划事件
- [ ] 终端显示局部路径姿态数量
- [ ] 终端同步显示速度指令变化

---

## 练习5: 多圈巡逻导航 (~15 分钟)

### 目标
使用 `followWaypoints` 实现多圈巡逻任务, 结合反馈判断每圈完成状态。

### 代码

源码位置: `navigation_lab/navigation_lab/waypoint_patrol.py`

```python
import math

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from .nav_utils import make_pose_stamped


def main():
    rclpy.init()
    nav = BasicNavigator()

    nav.setInitialPose(make_pose_stamped('map', 0.0, 0.0, 0.0))
    nav.waitUntilNav2Active()

    patrol_points = [
        make_pose_stamped('map', 2.0, 0.0, 0.0),
        make_pose_stamped('map', 2.0, 2.0, math.radians(90)),
        make_pose_stamped('map', 0.0, 2.0, math.radians(180)),
        make_pose_stamped('map', 0.0, 0.0, math.radians(-90)),
    ]

    loops = 2

    for lap in range(loops):
        nav.get_logger().info(f'=== Patrol lap {lap+1}/{loops} ===')

        nav.followWaypoints(patrol_points)

        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                nav.get_logger().info(
                    f'current_waypoint={feedback.current_waypoint}/{len(patrol_points)}'
                )

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            nav.get_logger().info(f'Lap {lap+1} complete!')
        else:
            nav.get_logger().error(f'Lap {lap+1} interrupted')
            break

    nav.get_logger().info('Patrol finished!')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 运行
```bash
ros2 run navigation_lab waypoint_patrol
```

### 观察
- 机器人完成 2 圈巡逻 (每圈 4 个航点)
- 终端显示 `current_waypoint` 从 `1/4` → `4/4`, 然后开始下一圈
- 如果某圈失败, 自动停止 (通过 `break`)

### 验收标准
- [ ] 机器人完成指定圈数的巡逻
- [ ] 每圈之间终端正确显示圈数信息
- [ ] 理解 `followWaypoints` 是阻塞式调用 — 必须在 `isTaskComplete()` 后检查结果

---

## 练习6: 综合挑战 — 自主巡逻 + 动态避障 (~20 分钟)

### 目标
结合 scan_injector 和 waypoint_patrol, 观察导航系统在动态障碍物环境中的表现。

### 步骤
```bash
# 终端1: Gazebo 仿真 (已运行)
# 终端2: Nav2 (已运行)
# 终端3: 巡逻程序
ros2 run navigation_lab waypoint_patrol
# 终端4: 启动后等待几秒, 再启动 scan_injector
ros2 run navigation_lab scan_injector
```

### 观察
1. 无干扰时: 机器人正常巡逻
2. 注入合成障碍物后机器人绕行:
   - `/plan` 全局路径重规划
   - `/local_plan` 局部轨迹调整
   - `/cmd_vel` 速度变化 (减速 + 转向)
3. 停止 scan_injector 后: 机器人恢复原路径

### 思考题
- `followWaypoints` 和循环 `goToPose` 的区别?
- 如果某个航点不可达, `stop_on_failure` 两种设置 (`true`/`false`) 的行为差异?
- 为什么 Gazebo 中 spawn 实体不显示在 LaserScan 中? (提示: `fake_laser` 的原理)

### 验收标准
- [ ] 动态障碍物注入后, 巡逻路径改变
- [ ] 障碍物移除后, 机器人恢复原计划
- [ ] 终端显示全局路径重规划和避障机动事件

---

## 常用调试命令

```bash
# 查看所有话题
ros2 topic list

# 查看导航 action 反馈 (实时)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}"

# 查看行为树状态
ros2 param get /bt_navigator default_nav_to_pose_bt_xml

# 查看生命周期节点状态
ros2 service call /bt_navigator/get_state lifecycle_msgs/srv/GetState "{}

# 手动清除代价地图
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"

# 查看机器人位姿
ros2 topic echo /amcl_pose

# 查看速度指令
ros2 topic echo /cmd_vel
```

## 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| `waitUntilNav2Active()` 卡住 | 未发布初始位姿 | 确保 `setInitialPose()` 在 `waitUntilNav2Active()` 之前调用, 或在 RViz 中用 2D Pose Estimate |
| 机器人不移动 | lifecycle 节点未激活 | 检查 launch 是否传了 `autostart:=true` |
| `followWaypoints` 反馈无 `distance_remaining` | `FollowWaypoints` action 只提供 `current_waypoint` | 改用 `NavigateToPose` 的 feedback 获取距离 |
| Gazebo 中实体不影响导航 | `fake_laser` 不响应 Gazebo 实体 | 使用 scan_injector 合成 LaserScan 消息 |
| 导航目标被拒绝 | 目标点在障碍物内或不可达 | 检查 RViz2 代价地图, 确认目标点在 free 区域 |
