# 第16章 PPT：Nav2 架构与核心组件

> 共 17 页，标注页码 · 图号与教学文档对应

---

## P1 标题页

# 第16章 Nav2 架构与核心组件

- 课程：ROS2 Python 编程
- 课时：2 课时（90 分钟）
- 教学方式：讲授 + 演示
- 主线：导航三大问题 → Nav2 四层架构 → 行为树导航 → 生命周期管理 → 核心组件 → 启动配置与演示

<!-- 旁白：各位同学大家好，欢迎进入第十六章。本章聚焦 ROS2 官方导航框架 Nav2，从架构、行为树、生命周期三大主线入手，认识它如何替代 ROS1 的 move_base。建议大家带着"导航要解决哪三个核心问题"的疑问开始学习，后续内容都围绕这条主线展开。 -->

---

## P2 学习目标

- 理解 Navigation2 整体架构和设计思想
- 掌握行为树驱动的导航决策框架
- 熟悉 ROS2 生命周期节点管理机制
- 了解 Nav2 核心组件（BT Navigator、Planner、Controller 等）及其功能
- 能够配置和启动 Nav2 导航系统
- 会使用 Simple Commander API 编写导航任务脚本

<!-- 旁白：本页的六项学习目标由宏观到微观排布：先建立架构观，再掌握行为树与生命周期两个核心机制，随后落到核心组件功能、配置启动，最后用 Simple Commander 编程收尾。前五项偏理解与操作，第六项需要动手实践，建议学完理论后在仿真环境中完整跑通一次导航任务再进入下一章。 -->

---

## P3 导航的三大问题

- **要点：** 定位、目标设定、规划与控制

机器人自主导航需要解决三个核心问题：

1. **我在哪里？——定位问题**：使用 AMCL 或 SLAM 确定机器人在地图中的位置；输入激光雷达、里程计、地图，输出机器人位姿估计
2. **我要去哪里？——目标设定**：由用户或任务系统指定目标位姿；可通过 RViz2 可视化设定，或由上层调度系统下发目标点
3. **我要怎么去？——规划与控制**：全局规划计算起点到终点的无碰撞路径；局部规划实时避障和路径跟踪；运动控制生成速度指令驱动机器人

<!-- 旁白：导航三大问题恰好对应机器人"我在哪里、要去哪里、怎么去"的完整决策闭环。定位通常由 AMCL 粒子滤波完成，目标设定来自用户或上层任务系统，规划与控制则拆成全局搜索与局部避障两层。三者的运行频率不同：定位与全局规划是低频任务，局部规划与运动控制运行在实时层，理解这一点有助于把握 Nav2 的分层设计。 -->

---

## P4 Nav2 概述与设计目标

- **要点：** 官方导航框架、插件化、行为树容错

**Nav2（Navigation2）** 是 ROS2 的官方导航框架，替代 ROS1 中的 move_base，采用行为树驱动的插件化架构。

设计目标：

- **模块化**：核心功能以插件形式提供
- **可扩展**：支持自定义规划器、控制器、恢复行为
- **鲁棒性**：行为树驱动的容错机制
- **资源高效**：通过生命周期管理按需启停组件

<!-- 旁白：Nav2 的设计目标可概括为四个关键词：模块化、可扩展、鲁棒性、资源高效。模块化依靠插件机制承载各项能力，可扩展允许用户自定义规划器、控制器与恢复行为，鲁棒性由行为树的容错机制保证，资源高效则靠生命周期管理实现按需启停。四个目标环环相扣，共同决定了 Nav2 的架构走向。 -->

---

## P5 Nav2 四层架构总览

- **要点：** 四层架构、NavigateToPose Action、数据流

```
用户接口层:  RViz2 | Simple Commander | 任务调度
                  ↓ 导航目标 (NavigateToPose Action)
        行为树导航器 (BT Navigator)
                  ↓                ↓
        Planner Server    Controller Server
        Dijkstra/A*       DWA/RPP/MPPI
                  ↓                ↓
        Global Costmap    Local Costmap
                  ↓
      传感器/里程计/地图/定位 (LaserScan, Odometry, Map, AMCL)
```

![图 16-w1：Nav2 系统架构](images/web/ch16/nav2_architecture.png)

图 16-w1：Nav2 系统架构（来源：docs.nav2.org）

<!-- 旁白：该图展示 Nav2 的完整数据流：用户经 RViz2 或 Simple Commander 下发目标，通过 NavigateToPose Action 交给行为树导航器，再分流给 Planner 与 Controller 两类服务器，最终落到代价地图与传感器定位层。注意自上而下的目标请求与自下而上的传感器反馈是并存的，这张总览图是理解后续章节的地图。 -->

---

## P6 Nav2 核心组件

- **要点：** 组件划分与职责、与 ROS1 的对应关系

| 组件 | 功能 | 对应 ROS1 |
|------|------|-----------|
| `bt_navigator` | 行为树导航决策引擎 | move_base |
| `planner_server` | 全局路径规划 | global_planner |
| `controller_server` | 局部路径规划与控制 | local_planner |
| `costmap_2d` | 全局/局部代价地图 | costmap_2d |
| `behavior_server` | 恢复行为管理 | recovery_behaviors |
| `amcl` | 粒子滤波定位 | amcl |
| `map_server` | 地图加载与服务 | map_server |
| `waypoint_follower` | 航点跟随 | - |

官方文档将 Nav2 描述为"一组松耦合、可独立替换的组件"：`bt_navigator` 只做决策编排，实际能力全部由 Action 服务器提供。

<!-- 旁白：这张对照表便于 ROS1 用户迁移学习：Nav2 的每个组件几乎都能在 move_base 体系里找到对应物，只有航点跟随是新功能。需要强调的是，bt_navigator 只做决策编排，规划、控制、恢复等实际能力全部由各 Action 服务器提供，组件间松耦合、可独立替换，这正是插件化架构的核心思想。 -->

---

## P7 Nav2 vs move_base

- **要点：** 从固定恢复流程到可编程决策

| 特性 | Nav2 (ROS2) | move_base (ROS1) |
|------|-------------|------------------|
| 架构 | 行为树驱动 | 有限状态机 |
| 可扩展性 | 插件化，高度可定制 | 有限的自定义 |
| 生命周期 | ROS2 生命周期节点 | 标准节点 |
| 多机器人 | 原生支持 | 需额外配置 |
| 恢复行为 | 可配置行为树 | 固定恢复序列 |
| 语言 | C++/Python | C++ |

关键区别：恢复行为不再是一串写死的 fallback，而是行为树中的一等公民节点，可按场景裁剪。

<!-- 旁白：本表揭示了 Nav2 与 move_base 最本质的差异在决策模型：前者用行为树表达导航策略，后者是固定的有限状态机。因此恢复行为从写死的失败处理序列变成行为树中的节点，可随场景裁剪，例如只保留旋转与清理代价地图。从工程角度看，这种可编程决策为复杂导航任务留出了充分的定制空间。 -->

---

## P8 行为树基础

- **要点：** 控制节点、装饰节点、执行节点

行为树（Behavior Tree）是用于控制决策流程的树形结构：

- **控制节点**：Sequence（顺序执行，全部成功才成功）、Fallback（任一成功则成功）、Parallel（并行执行）
- **装饰节点**：Repeater（重复）、Inverter（反转）、RateController（限频）、Condition（条件检查）
- **执行节点**：Action（执行导航、旋转等具体任务）、Condition（检查是否到达目标等）

| 特性 | 行为树 | 有限状态机 |
|------|--------|-----------|
| 模块化 | 高，节点可重用 | 低，状态转换耦合 |
| 可读性 | 树形结构直观 | 状态图复杂 |
| 调试 | 可视化 XML | 日志跟踪 |
| 容错 | 自然支持 | 需显式设计 |

<!-- 旁白：行为树把决策拆成控制、装饰、执行三类节点：控制节点决定分支走向，装饰节点附加重复、限频等策略，执行节点完成具体动作。与有限状态机相比，行为树的节点可复用、树形结构直观、支持 XML 可视化调试，容错由节点组合自然表达，这正是 Nav2 选择行为树作为决策框架的根本原因。 -->

---

## P9 Nav2 默认行为树与执行流程

- **要点：** navigate_to_pose.xml、PipelineSequence、RecoveryNode

默认导航行为树 `navigate_to_pose.xml` 的执行流程：

```
接收到导航目标 {goal}
  └─ PipelineSequence: NavigateWithRecovery
       ├─ 第一阶段 RateController(1Hz): 全局规划
       │    ├─ ComputePathToPose 计算全局路径
       │    └─ 失败 → RecoveryNode: ClearEntireCostmap 后重试
       └─ 第二阶段 RateController(10Hz): 路径跟踪
            ├─ FollowPath 执行跟踪（10Hz 重规划）
            └─ 失败 → RecoveryNode: Spin → Wait → BackUp
                      → ClearEntireCostmap → 重试 FollowPath
```

- `PipelineSequence`：前序分支保持运行、后续分支并发推进（支持跟踪中 1Hz 重规划）
- `nav2_behavior_tree` 包内置约 50 个现成节点（`IsPathValid`、`GoalUpdated`、`ClearCostmapService` 等），可用 Groot 可视化编辑

<!-- 旁白：这份流程值得逐层解读：外层 PipelineSequence 先以 1Hz 频率规划全局路径，成功后转入 10Hz 的路径跟踪，两个分支还能并发推进。每个阶段各自挂接 RecoveryNode，失败时依次执行清理代价地图或旋转、等待、后退等恢复动作后重试，这套"重规划加恢复"机制就是 Nav2 容错导航的内核。 -->

---

## P10 生命周期节点管理

- **要点：** 状态机、生命周期管理器、一键启动

Nav2 所有导航组件均为生命周期节点（Lifecycle Node），具有明确的状态转换：

![图 16-w2：ROS2 生命周期节点状态机](images/web/ch16/lifecycle_sm.png)

图 16-w2：ROS2 生命周期节点状态机（来源：design.ros2.org）

- 状态：Unconfigured → Inactive → Active → Finalized
- 转换：`configure` / `activate` / `deactivate` / `cleanup` / `shutdown`
- 常用命令：`ros2 lifecycle get <节点>`、`ros2 lifecycle set <节点> activate`
- 生命周期管理器可一键启动：创建所有节点 → 依次 configure → 依次 activate，未就绪即回滚，保证系统鲁棒性与资源高效利用

<!-- 旁白：状态机图展示了生命周期节点的四个状态与转换动作：先 configure 完成初始化，再 activate 激活，运行中可 deactivate 暂停、cleanup 清理、shutdown 收尾。configure 与 activate 分离的用意在于组件可先统一初始化、校验就绪后再整体激活，配合生命周期管理器的一键启动与失败回滚，实现机器人上鲁棒且节省资源的运行管理。 -->

---

## P11 Planner Server 与 Controller Server

- **要点：** 全局规划 ComputePathToPose、局部跟踪 FollowPath、cmd_vel

**Planner Server（全局规划）**：在全局代价地图上搜索起点到终点的无碰撞路径，支持 Navfn（Dijkstra/A*）、SMAC（含 Hybrid-A*，支持阿克曼底盘）等插件，通过 `ComputePathToPose` Action 提供服务。

**Controller Server（局部规划与控制）**：跟踪全局路径并避开动态障碍物，实时处理激光数据并输出 `cmd_vel` 速度指令。

| 插件 | 类型 | 特点 | 适用场景 |
|------|------|------|----------|
| DWB | 采样式 | 经典 DWA 实现，参数直观 | 差速底盘入门 |
| MPPI | 采样式（MPC） | 无约束轨迹优化，平滑高效 | 高动态场景 |
| RPP | 几何跟踪 | regulated pure pursuit，限速安全 | 简单可靠的巡线 |

<!-- 旁白：Planner 与 Controller 一高一低：前者在全局代价地图上搜索无碰撞路径，后者跟踪路径并实时避障，输出 cmd_vel 速度指令。表中三种局部控制器各有侧重：DWB 参数直观适合差速底盘入门，MPPI 用无约束轨迹优化生成平滑轨迹适合高动态场景，RPP 以受调节的纯跟踪实现安全限速适合巡线，选型主要权衡计算量与平滑性。 -->

---

## P12 Behavior Server 与代价地图层

- **要点：** 恢复行为、代价地图分层建模

**Behavior Server（恢复行为）**：管理导航失败时的恢复策略，包括 Spin（原地旋转）、BackUp（后退）、Drive on Heading（沿当前方向前进）、Wait（等待）、Clear Costmap（清除代价地图）。

**代价地图层**：将环境建模为带代价值的栅格：

| 图层 | 插件 | 功能 |
|------|------|------|
| 静态层 | StaticLayer | 加载预建地图 |
| 障碍物层 | ObstacleLayer | 传感器实时障碍物 |
| 膨胀层 | InflationLayer | 障碍物膨胀 |
| 体素层 | VoxelLayer | 3D 障碍物感知 |

全局代价地图偏重静态层+膨胀层（离线规划），局部代价地图偏重障碍物层+滚动窗口（在线避障）。

<!-- 旁白：Behavior Server 兜底管理导航失败时的恢复策略，而代价地图把环境建模为带代价值的栅格。表中四个图层插件各司其职：静态层加载预建地图，障碍物层实时融合传感器观测，膨胀层扩大安全距离，体素层感知三维障碍。全局与局部代价地图的图层侧重不同，正对应离线规划与在线避障两种用途。 -->

---

## P13 Nav2 启动与配置

- **要点：** nav2_bringup、nav2_params.yaml、Simple Commander

```bash
# 使用预建地图启动导航
ros2 launch nav2_bringup navigation_launch.py \
  map:=./maps/office_map.yaml \
  params_file:=./config/nav2_params.yaml \
  use_sim_time:=true

# 自动启动（包含定位）
ros2 launch nav2_bringup bringup_launch.py \
  map:=./maps/office_map.yaml use_sim_time:=true
```

- `nav2_params.yaml` 按节点名分段配置：`bt_navigator`、`planner_server`、`controller_server`、`global_costmap`、`local_costmap`、`amcl` 等
- **Simple Commander API**（生产级封装）：`BasicNavigator` + `waitUntilNav2Active()` 等待就绪 → `goToPose(goal)` 发送目标 → `getFeedback()` 监控剩余距离 → `getResult()` 检查结果；还提供 `goThroughPoses`、`followWaypoints` 等多目标接口

<!-- 旁白：启动命令的关键在参数：map 指定预建地图，params_file 载入按节点名分段编写的配置，带 bringup 的脚本还会自动启动定位节点。Simple Commander 的编程范式固定：先等待 Nav2 各组件就绪，再发送目标、周期读取反馈并检查最终结果，另有 goThroughPoses、followWaypoints 等接口可组合出复杂的多目标任务。 -->

---

## P14 运行演示

- **要点：** 启动 Nav2 组件、观察生命周期状态

演示步骤：

1. 启动仿真环境与 Nav2 导航栈
2. `ros2 lifecycle list` 查看 Nav2 组件的生命周期状态
3. 在 RViz2 中使用 Navigation2 设定目标点，观察全局路径与局部轨迹
4. 人为遮挡目标点，观察 Spin/BackUp/Clear Costmap 恢复行为

![运行演示：ch11 Nav2 组件启动](images/runtime/nonlab_nav2.gif)

<!-- 旁白：演示环节注意两个观察点：一是执行 ros2 lifecycle 命令时各组件状态从 Unconfigured 逐一切换到 Active 的过程；二是在 RViz2 中设定目标后机器人沿全局路径行驶并实时局部避障，遇阻时 Spin、BackUp、Clear Costmap 的触发顺序与行为树配置一一对应，可回到 P9 的流程图对照理解。 -->

---

## P15 本章要点

- 导航三大问题：我在哪里（定位）、我要去哪里（目标设定）、我要怎么去（规划与控制）
- Nav2 是 ROS2 官方导航框架，采用行为树驱动的插件化架构，四层结构清晰
- 行为树通过 PipelineSequence、RecoveryNode、RateController 实现"重规划 + 恢复"的容错导航
- 生命周期节点提供明确的状态机，生命周期管理器支持一键有序启动
- 核心组件：BT Navigator 决策编排，Planner/Controller/Behavior Server 各司其职
- 代价地图分层建模：静态层、障碍物层、膨胀层、体素层
- Simple Commander API 是生产级 Python 封装，适合编写业务导航逻辑

<!-- 旁白：本章要点可浓缩为一句话：Nav2 以行为树做决策、以生命周期管启停、以插件承载能力。七条要点覆盖从三大问题的提出到 Simple Commander 编程的完整知识链，其中四层架构、行为树容错与代价地图分层是后续章节反复使用的基础，建议对照 P5 的架构图重点记忆。 -->

---

## P16 练习题

1. **架构题**：绘制 Nav2 的系统架构图，说明各组件之间的数据流和依赖关系。
2. **配置题**：编写 `nav2_params.yaml`，包括行为树导航器、全局规划器、局部控制器和代价地图的完整配置。
3. **编程题**：使用 `nav2_simple_commander` 编写多点导航程序，机器人依次访问 5 个目标点，每到达一个点后暂停 3 秒。
4. **分析题**：分析 Nav2 使用生命周期节点管理的优势，说明状态转换机制如何提高系统鲁棒性和资源利用效率。
5. **设计题**：设计一个自定义导航行为树，包含电池检查、前往充电桩、充电等待、继续任务，写出对应 XML。

<!-- 旁白：五道练习题由浅入深：前两题考察架构图绘制与参数文件编写，第三题要求用 Simple Commander 实现五点巡航并间隔停留，第四题分析生命周期管理对鲁棒性与资源效率的价值，第五题综合设计带电池检查与回充的行为树。最后一题最能检验本章掌握程度，建议课后动手写出完整 XML。 -->

---

## P17 下章预告

**第17章 全局代价地图**

- 代价地图的分层结构与数据流
- 静态层、障碍物层、膨胀层的工作原理与参数调优
- 全局/局部代价地图的配置差异
- 传感器数据融合与滚动窗口
- 代价地图常见问题排查（穿墙、膨胀过大、更新卡顿）

> 预习建议：回顾第16章 16.5.5 节代价地图层内容，并尝试在 RViz2 中显示 costmap 话题。

<!-- 旁白：本章建立了 Nav2 的整体框架，但代价地图的分层建模只是作了概述。下一章将聚焦全局代价地图，深入图层插件、膨胀机制与参数调优，并对比全局与局部代价地图的差异，这些内容是学习第十八章路径规划算法的必要前提，建议先回顾 P12 的表格做好衔接。 -->
