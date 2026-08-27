# 第11章 Nav2 自主导航

## 第1页: 章节目录
- 11.1 Nav2 系统架构
- 11.2 行为树 (Behavior Tree)
- 11.3 全局规划器 (Planner)
- 11.4 局部控制器 (Controller)
- 11.5 代价地图 (Costmap2D)
- 11.6 恢复行为 (Recovery)
- 11.7 Simple Commander API
- 11.8 导航系统集成

---

## 第2页: Nav2 架构总览

```
┌─────────────────────────────────────────────────┐
│                   Nav2 架构                       │
│                                                   │
│  ┌──────────┐    ┌──────────────┐                │
│  │ 全局规划器 │<───│  行为树引擎   │                │
│  │ (Planner) │    │  (BT Engine) │                │
│  └─────┬─────┘    └──────┬───────┘                │
│        │ 全局路径         │ 执行控制               │
│  ┌─────▼──────────────────▼───────┐              │
│  │       局部控制器 (Controller)    │              │
│  │    DWB / MPPI / RPP / TEB      │              │
│  └───────────────┬────────────────┘              │
│                  │ cmd_vel                        │
│  ┌───────────────▼────────────────┐              │
│  │   机器人基座 (Robot Base)       │              │
│  └────────────────────────────────┘              │
│                                                   │
│  ┌──────────┐  ┌──────────────┐                  │
│  │全局代价地图│  │ 局部代价地图   │                  │
│  │(Static)  │  │ (Rolling)    │                  │
│  └──────────┘  └──────────────┘                  │
└─────────────────────────────────────────────────┘
```

---

## 第3页: 行为树 vs 有限状态机

```
FSM (传统):
   [IDLE] --goal--> [PLAN] --path--> [FOLLOW] --done--> [IDLE]
     ^                  |                                      |
     |                  | fail                                |
     +------------------+                                     |
     +-------------------------------------------------------+

Behavior Tree (Nav2):
  NavigateToPose
    └── Sequence
         ├── ComputePathToPose
         │    └── Recovery: 失败时 ClearCostmap → retry
         └── FollowPath
              └── Recovery: 失败时 Spin → Wait → ClearCostmap → retry
```

**优势:** 模块化、可复用、运行时可变、可视化调试

---

## 第4页: 行为树节点类型

| 节点 | 图标 | 语义 | 返回 |
|------|------|------|------|
| Sequence | → | 顺序执行子节点 | 任一 FAILURE 则 FAILURE |
| Fallback | ? | 尝试子节点 | 任一 SUCCESS 则 SUCCESS |
| Parallel | ⇉ | 并行执行 | 策略: N of M |
| ReactiveFallback | ¿ | 可中断 | 高优先级子节点可抢占 |
| Decorator | ◇ | 修饰子节点 | 依赖装饰器类型 |
| Action | ◆ | 执行动作 | 返回 RUNNING/SUCCESS/FAILURE |
| Condition | ◯ | 条件检查 | SUCCESS/FAILURE |

**Nav2 自定义节点示例:**
```
◆ ComputePathToPose  ◆ FollowPath  ◆ Spin  ◆ Wait  ◆ BackUp
◆ DriveOnHeading  ◆ AssistedTeleop  ◆ SmoothPath
◯ GoalUpdated  ◯ isBatteryLow  ◯ isPathValid  ◯ TransformAvailable
```

---

## 第5页: 全局规划器 (1/2)

**Plan = f(start, goal, costmap) → Path**

```
  ┌───┬───┬───┬───┬───┐   规划结果 (路径可视化):
  │ S │   │   │   │ ■ │      S──>──>──>──┐
  ├───┼───┼───┼───┼───┤                  │
  │   │ ■ │ ■ │   │ ■ │                  >──>──>──>──> G
  ├───┼───┼───┼───┼───┤
  │   │ ■ │   │   │   │   S: 起点  G: 终点  ■: 障碍物
  ├───┼───┼───┼───┼───┤
  │   │   │   │ ■ │ G │
  └───┴───┴───┴───┴───┘
```

**A* 算法伪代码:**
```
OPEN = {start}, CLOSED = {}
while OPEN:
    n = argmin(OPEN, key=g(n) + h(n))
    if n == goal: return reconstruct_path(n)
    移入 CLOSED
    for neighbor in neighbors(n):
        计算 tentative_g
        if tentative_g < g(neighbor):
            parent[neighbor] = n
            更新 OPEN
return FAILURE
```

---

## 第6页: 全局规划器选型 (2/2)

| 规划器 | 算法 | 动力学 | 速度 |
|--------|------|--------|------|
| NavFn | 导航函数 | 无约束 | 快 |
| SmacHybrid | Hybrid-A* | Dubins/Reeds-Shepp | 中 |
| SmacLattice | 状态格 | 全动力学 | 中 |
| ThetaStar | Theta* | 无约束 | 快 |

**配置示例 (nav2_params.yaml):**
```yaml
planner_server:
  planner_plugins: ["GridBased"]
  GridBased:
    plugin: "nav2_smac_planner/SmacPlannerHybrid"
    tolerance: 0.5
    downsample_costmap: false
    allow_unknown: true
    max_iterations: 1000000
    angle_quantization_bins: 72
```

---

## 第7页: 局部控制器 DWB

**DWB (Dynamic Window approach Based):**

```
每个控制周期:
  1. 采样候选速度 (v, ω) 组合
     ┌──────────────────────┐
     │ v ∈ [v_min, v_max],  │
     │ ω ∈ [ω_min, ω_max]   │
     └──────────────────────┘
  2. 前向模拟 t_fwd 秒的轨迹
  3. 每个 Critic 评分 → 加权求和
  4. 选择最小代价的速度指令 → cmd_vel
```

**DWB Critics (评分器):**
```
GoalDist   ─── 目标距离
GoalAlign  ─── 目标方向对齐
PathDist   ─── 全局路径偏离
PathAlign  ─── 路径方向对齐
Obstacle   ─── 障碍物距离 (安全)
Oscillation─── 震荡抑制
RotateToGoal── 原地旋转到目标方向
```

---

## 第8页: 代价地图架构

```
Master Costmap (融合)
│
├── StaticLayer      ← 预建地图 (OccupancyGrid)
├── ObstacleLayer    ← 传感器实时更新 (LaserScan/PointCloud)
├── InflationLayer   ← 障碍物膨胀 (安全半径)
├── VoxelLayer       ← 3D 体素标记 (RGB-D/LiDAR)
├── RangeLayer       ← 距离传感器直接投影
├── KeepoutFilter    ← 禁区/限行区域
└── SpeedFilter      ← 限速区域
```

**代价计算:**
```
static(占据) + inflation(衰减) = final_cost
     100         +    253       =    253  (LETHAL_OBSTACLE)
      50          +    128       =    178  (INSCRIBED_INFLATED)
```

**重要参数:**
```
inflation_radius:    0.55 m   (膨胀半径)
cost_scaling_factor: 3.0     (代价衰减因子)
robot_radius:        0.22 m   (机器人半径)
```

---

## 第9页: 全局 vs 局部代价地图

| 特性 | 全局代价地图 | 局部代价地图 |
|------|------------|-------------|
| 坐标系 | map | odom/base_link |
| 更新频率 | 1~2 Hz | 5~10 Hz |
| 大小 | 整个地图 | 滚动窗口 (3x3m) |
| 窗口模式 | 固定 | 跟随机器人滚动 |
| 传感器更新 | 否 (或低频) | 是 (高频) |
| 用途 | 全局路径规划 | 局部避障 |
| 分辨率 | 0.05 m (可粗) | 0.05 m (需细) |

```
  ┌────────────────────┐  ← 全局代价地图 (全图, 低频)
  │    ┌───────┐       │
  │    │局部窗口│       │  ← 局部代价地图 (滚动, 高频)
  │    │  ┌─┐  │       │
  │    │  │R│  │       │  R = 机器人
  │    │  └─┘  │       │
  │    └───────┘       │
  └────────────────────┘
```

---

## 第10页: 恢复行为链

```
NavigateToPose
  └── RecoveryNode ──────────────────────────
       ├── ComputePathToPose (主行为)
       │    └── 失败时 → 进入恢复序列
       └── Sequence (恢复链)
            ├── ClearGlobalCostmap     ← 清除全局代价地图
            ├── Spin                   ← 原地旋转 360°
            ├── Wait (2s)              ← 等待传感器更新
            ├── BackUp (0.3m)          ← 后退
            ├── ClearLocalCostmap      ← 清除局部代价地图
            └── Wait (1s)              ← 最后等待

  不论恢复成功/失败: 重新尝试 ComputePathToPose
  恢复失败 → 导航任务失败
  恢复成功 → 路径仍有效 → FollowPath
```

**RecoveryNode 的有效循环:**
```
navigate_to_pose (失败) → 恢复1→ 恢复2→ ...
    如果所有恢复都失败 → 任务失败
    如果某个恢复成功 → 重新规划
```

---

## 第11页: Simple Commander API

```
导航 → goToPose(goal_pose)
         └── feedback: distance_remaining, estimated_time_remaining
         └── result: SUCCEEDED / FAILED / CANCELED

航点 → followWaypoints([pose1, pose2, ...])
         └── feedback: current_waypoint (当前航点编号)

周期 → followPath(path)           # 执行预计算路径
       cancelTask()                # 取消当前任务
       clearAllCostmaps()          # 清除所有代价地图
       isTaskComplete()            # 检查任务状态
       lifecycleStartup()          # 生命周期启动
       lifecycleShutdown()         # 生命周期停止
```

**回调注册 (用于中途干预):**
```python
# 注册全局 Localization 回调
navigator.waitUntilNav2Active(
    navigator='bt_navigator',
    localizer='amcl'
)
```

---

## 第12页: 自定义行为树

**通过 XML 完全自定义导航行为:**

```xml
<root>
  <BehaviorTree ID="CustomNavigate">
    <PipelineSequence name="NavigateWithRetry">
      <!-- 1. 路径规划 -->
      <RecoveryNode number_of_retries="3">
        <ComputePathToPose goal="{goal}" planner_id="GridBased"/>
        <Sequence>
          <ClearEntireCostmap service_name="local_costmap/clear"/>
          <Wait wait_duration="1.0"/>
        </Sequence>
      </RecoveryNode>
      <!-- 2. 路径跟随 -->
      <RecoveryNode number_of_retries="2">
        <FollowPath path="{path}" controller_id="FollowPath"/>
        <Sequence>
          <Spin spin_dist="3.14159"/>
          <BackUp backup_dist="0.3"/>
          <ClearEntireCostmap service_name="global_costmap/clear"/>
        </Sequence>
      </RecoveryNode>
    </PipelineSequence>
  </BehaviorTree>
</root>
```

---

## 第13页: 导航启动完整流程

```bash
# 完整导航系统启动 (4 个终端)
# T1: Gazebo 仿真
ros2 launch xbot_sim xbot_gazebo.launch.py

# T2: Nav2 全部组件
ros2 launch nav2_bringup navigation_launch.py \
  map:=src/maps/office_map.yaml \
  params_file:=src/config/nav2_params.yaml \
  use_sim_time:=true

# T3: rviz 可视化
rviz2 -d src/config/nav2_view.rviz

# T4: Python 导航任务 (nav_goal_runner.py)
python3 nav_goal_runner.py
```

**Navigation launch 自动启动的组件:**
```
controller_server, planner_server, behavior_server,
bt_navigator, waypoint_follower, velocity_smoother,
lifecycle_manager, map_server, amcl
```

---

## 第14页: 本章总结

**核心要点:**
1. Nav2 采用行为树架构, 可定制运行逻辑
2. 全局规划器计算地图上的最优路径
3. 局部控制器 DWB/MPPI 处理实时避障
4. 代价地图分层感知障碍物与安全区域
5. 恢复行为提供机器人卡住时的自救能力
6. Simple Commander API 大幅简化 Python 调用

**调试命令:**
```bash
ros2 param list /controller_server      # 查看控制器参数
ros2 run nav2_util lifecycle_bringup    # 生命周期管理
ros2 topic echo /plan                   # 查看全局规划结果
ros2 topic echo /local_plan             # 查看局部规划结果
```

**下一步: 第12章 MoveIt 2 机械臂编程**