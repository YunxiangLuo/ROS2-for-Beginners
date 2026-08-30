# 第29章 PPT：抓取与放置编程

> 共 17 页，标注页码 · 图号与教学文档对应

---

## P1 · 标题页

- **课程：** ROS2 机器人编程
- **章节：** 第 29 章 抓取与放置编程
- **课时：** 2 课时
- **内容：** 抓取姿态计算、夹爪控制、attach/detach 物体管理、Pick-and-Place 十步流程、多物体分拣与码垛、MoveIt Task Constructor 阶段化流水线

<!-- 旁白：上一章解决了笛卡尔路径与避障，本章把视角转向机械臂最经典的工业任务：抓起一个物体并放到指定位置；先讲抓取姿态与夹爪，再讲物体附着与完整流程，最后引入官方的阶段化方案 MTC。 -->

---

## P2 · 学习目标

- **要点：**
- 掌握抓取姿态的计算方法，会构造预抓取、抓取与撤离位姿链
- 理解顶部抓取与侧面抓取的适用场景与姿态差异
- 学会控制夹爪的开合运动与力矩/速度受限抓取
- 理解 attach/detach 物体的 API 与规划场景语义
- 掌握 Pick-and-Place 完整流程与多物体分拣、码垛实现
- 了解官方 MoveIt Task Constructor 阶段化流水线的思想

<!-- 旁白：本章目标围绕"抓得住、放得准"展开：位姿链与夹爪保证抓得住，attach/detach 与十步流程保证放得准，MTC 则给出工程化的组织方式。 -->

---

## P3 · 抓取姿态分析

- **要点：** 抓取姿态由四个要素决定：接近方向、抓取角度、抓取高度与撤离方向，核心公式为 `T_grasp = T_object × T_offset`

```python
grasp_pose = deepcopy(object_pose)   # 抓取位姿 = 物体位姿 + 偏移
grasp_pose.position.z += gripper_depth

approach_pose = deepcopy(grasp_pose) # 接近预位姿：上方 10 cm
approach_pose.position.z += 0.1

retreat_pose = deepcopy(grasp_pose)  # 撤离位姿：上方 15 cm
retreat_pose.position.z += 0.15
```

- 三位姿链：approach → grasp → retreat，是所有抓取程序的骨架
- `compute_grasp_pose(object_pose, approach_angle, approach_distance, grasp_height_offset)` 为通用入口

<!-- 旁白：先建立"位姿链"的概念：真正抓取的那一位姿只是链条一环，前后还要有接近与撤离位姿，这也解释了为什么抓取程序总有"先到上方、再下降、再抬起"的固定节奏。 -->

---

## P4 · 顶部抓取与侧面抓取

- **要点：** 顶部抓取末端朝下从上方接近，侧面抓取沿水平方向从侧面夹取，选择依据是物体的形状与可接近面

| 抓取方式 | 末端姿态（欧拉角） | 适用物体 | 接近方式 |
| --- | --- | --- | --- |
| 顶部抓取 | (π, 0, 0)，末端朝下 | 顶部平坦的盒体、桌面物体 | 沿 -Z 下降，z 加 gripper_depth |
| 侧面抓取（X 向） | (0, π/2, 0) | 侧面平整、顶部不便接近 | 沿 -X 水平接近 0.1 m |
| 侧面抓取（Y 向） | (π/2, 0, 0) | 同上，换接近面 | 沿 -Y 水平接近 0.1 m |

- 顶部抓取用 `quaternion_from_euler(math.pi, 0, 0)` 直接构造朝下姿态
- 练习第 1 题要求手动构造两类位姿，并分析各自适用的物体形状

<!-- 旁白：抓取方式的本质是选择接近方向：朝下接近最稳但要求顶部可达，侧面接近适合顶部被遮挡或需要避免下压的物体；两种姿态都由欧拉角转四元数生成。 -->

---

## P5 · 官方要点：GraspIt!、GPD 与 Deep Grasp

- **要点：** 官方生态提供两条抓取位姿生成路线：GraspIt! 基于物体 mesh 与手掌模型枚举力闭合抓取，GPD 用 CNN 从点云直接预测候选抓取位姿

- GraspIt!（哥伦比亚大学仿真器）：输出带抓取分数的候选列表，适合离线分析
- GPD（Grasp Pose Detection）：从点云实时预测，输出 MoveIt 可用的位姿列表
- 官方 Deep Grasp 文档把"感知 → 抓取候选 → IK 验证 → 执行"串成完整示例
- 官方建议：对候选位姿按"可达性 + 碰撞 + 接近方向可达"排序后再逐个尝试

<!-- 旁白：本章的手工位姿计算是理解的起点，工程上更多用抓取检测算法自动生成候选；GraspIt! 偏仿真分析，GPD 偏实时感知，两者都最终汇入同一套验证与执行流程。 -->

---

## P6 · 夹爪控制

- **要点：** 夹爪在 MoveIt2 中是独立的规划组，开合就是给关节组设置关节目标并规划执行

```python
self.gripper = PlanningComponent(
    self.moveit, 'gripper', 'gripper_link'
)

def open_gripper(self, width=0.65):
    self.gripper.set_start_state_to_current_state()
    self.gripper.set_joint_value_target([width, width])
    plan = self.gripper.plan()
    if plan:
        self.gripper.execute(plan.trajectory)
```

| 动作 | 关节目标 | 说明 |
| --- | --- | --- |
| 完全打开 | [0.65, 0.65] | 双指各到最大开度 |
| 半开 | [0.3, 0.3] | 中间过渡位 |
| 完全闭合 | [0.0, 0.0] | 接触并夹紧物体 |
| 模拟抓取 | 开 width+0.01 再闭 -0.01 | 先比物体略宽，再闭合 |

<!-- 旁白：夹爪虽然小，却完整走了"设目标、规划、执行"的流程，与臂部规划完全同构；抓取物体时先开到比物体略宽 1 厘米再闭合，给定位留出余量。 -->

---

## P7 · 夹爪力矩和速度控制

- **要点：** 通过缩放因子限制夹爪速度与加速度，并用"分步执行"模拟力矩控制的缓闭过程

```python
def controlled_grasp(self, max_effort=50.0):
    self.gripper.set_max_velocity_scaling_factor(0.3)
    self.gripper.set_max_acceleration_scaling_factor(0.3)
    self.gripper.set_joint_value_target([0.0, 0.0])
    plan = self.gripper.plan()
    if plan:
        trajectory = plan.trajectory
        # 只执行前半段轨迹（缓慢接近）
        mid_point = len(trajectory.points) // 2
        trajectory.points = trajectory.points[:mid_point]
        self.moveit.execute(plan)
```

- 缩放因子 0.3 意味着以 30% 的最大速度/加速度运动，降低冲击
- 注意：moveit_py 的执行接口挂在 `MoveItPy` 对象上，而非 `PlanningComponent`
- 执行依赖执行服务器在线，课程统一封装 `ensure_execution_servers` 提供有界等待与明确报错

<!-- 旁白：真正的力控需要力传感器反馈，这里用"限速 + 截断轨迹"模拟缓慢接近的过程，重点是理解执行接口的归属：plan 在 PlanningComponent 上，execute 在 MoveItPy 对象上。 -->

---

## P8 · Attach/Detach 物体

- **要点：** 抓住物体后用 `AttachedCollisionObject` 把它附着到末端连杆，物体随末端运动并自动获得自碰撞放行

```python
aco = AttachedCollisionObject()
aco.link_name = link_name                    # 附着到夹爪连杆
aco.object = co                              # CollisionObject 本体
aco.touch_links = ['gripper_link', 'link5']  # 允许接触的连杆

self.psm.process_attached_collision_object(aco)   # 附着
self.psm.process_attached_collision_object(aco_remove)  # 分离：REMOVE
```

- 附着时物体坐标系挂在 `link_name` 下，`touch_links` 声明允许接触的连杆，避免夹爪与物体被误判碰撞
- 分离是镜像操作：`co.operation = REMOVE` 后重新处理即可，物体回到场景
- 附着后物体进入 Allowed Collision Matrix 自动放行列表，可贴着抓手运动

<!-- 旁白：attach/detach 是上一章规划场景知识的直接延伸：附着把物体"长"在末端上参与所有碰撞检查，分离则把物体交还给场景，一对操作支撑起整个抓放语义。 -->

---

## P9 · Pick-and-Place 十步流程

- **要点：** 标准抓放流程共十步，抓取段与放置段结构对称，attach/detach 分别是两段的转折点

| 步骤 | 动作 | 说明 |
| --- | --- | --- |
| 1 | 打开夹爪 | 张开到 0.65 |
| 2 | 移动到预抓取位姿 | 物体上方 0.12 m |
| 3 | 下降到抓取位姿 | 到达物体位置 |
| 4 | 闭合夹爪 | 抓住物体 |
| 5 | 附着物体 | attach 加入规划场景 |
| 6 | 撤离 | 上升 0.1 m 到安全高度 |
| 7 | 移动到预放置位姿 | 放置点上方 0.1 m |
| 8 | 下降到放置位姿 | 到达放置位置 |
| 9 | 打开夹爪并分离物体 | 释放 + detach |
| 10 | 撤离回安全位置 | 最后回到 home 位姿 |

- 示例场景：桌子 0.8×0.6×0.02，抓取点 (0.3, 0.1, 0.15)，放置点 (0.3, -0.2, 0.15)
- 每步之间 sleep 0.3~0.5 s，给规划场景更新与夹爪动作留出时间

<!-- 旁白：十步流程是本章的骨架代码：前六步 pick、后五步 place，attach 与 detach 恰好是两段的分界；把这个节奏背下来，任何抓放任务都只是在这条时间线上填入具体位姿。 -->

---

## P10 · 官方要点：三段式抓取状态机

- **要点：** 官方教程把十步流程压缩为三段式范式：pre-grasp approach、grasp、post-grasp retreat，放置段与之对称

- pre-grasp approach：沿撤离方向的反向接近物体
- grasp：夹爪闭合 + attach，物体瞬间进入规划场景并随末端连杆运动
- post-grasp retreat：撤离到安全高度，等待转移
- attach/detach 并非透明魔法：官方用 `ModifyPlanningScene` service 实现
- 夹爪通过 `GripperCommand` action 控制，规划器全程只负责臂部轨迹

<!-- 旁白：官方范式与本章代码一一对应：三段式就是三位姿链，ModifyPlanningScene 就是 process_attached_collision_object 的底层服务，理解了这层映射，读官方 pick_place_task 源码就不会迷路。 -->

---

## P11 · 多物体分拣

- **要点：** 多物体分拣用"物体位置字典 + 放置区域字典"组织，按物体属性路由到对应放置区

```python
def sort_objects(self, object_locations, place_zones):
    for obj_id, obj_pose in object_locations.items():
        self.pick(obj_pose)              # 逐个抓取

        place_pose = place_zones.get('default')
        if 'red' in obj_id:              # 按颜色路由
            place_pose = place_zones.get('red_zone')
        elif 'blue' in obj_id:
            place_pose = place_zones.get('blue_zone')

        self.place(place_pose)           # 放置到对应区域
```

- 循环体就是完整的 pick + place，前一个物体的放置结果是下一个物体规划的初始场景
- 分拣难点在路径干涉：连续抓放要保证轨迹互不干扰

<!-- 旁白：把单次抓放封装成 pick/place 方法后，多物体分拣只是在外层加一个路由循环；颜色分区是任务级逻辑，真正难的工程问题往往是避免前一次放置挡住下一次抓取。 -->

---

## P12 · 官方要点：MoveIt Task Constructor（MTC）

- **要点：** 官方用 MTC 把 pick-and-place 建模为串联的阶段流水线：GenerateGraspPose → ComputeIK → ModifyPlanningScene → MoveTo，各阶段独立验证、可并行生成多个解

![MTC stage types: moveit.picknik.ai](images/web/ch29/ch29_mtc_stage_types.png)

MTC 的阶段类型一览

- 失败时按顺序回溯，而不是整条流程重跑
- `SerialContainer` 可逐物体实例化整条流水线，天然适合分拣与码垛
- 官方示例用 `current_state` 阶段锚定 home 位姿，避免写死关节值

<!-- 旁白：MTC 的核心思想是"分而治之"：每个阶段只解决一件事并输出解集合，下游阶段对解做过滤与传递；这张官方图列出了生成器、传播器、过滤器等阶段类型，是读懂 MTC 程序的地图。 -->

---

## P13 · MTC 流水线可视化

- **要点：** 官方 pick_place_task 示例在 RViz 的 Task Constructor 面板中逐阶段展示解，直观呈现"抓取—搬移—放置"的分解

![MTC stages: moveit.picknik.ai](images/web/ch29/ch29_stages.png)

MTC 任务阶段在 RViz 面板中的显示

![MTC first stages: moveit.picknik.ai](images/web/ch29/ch29_first_stages.png)

pick_place_task 前几个阶段的可视化

- 每个阶段右侧显示解的数量与代价，可单独调试某个阶段
- 阶段化让失败定位变得容易：哪个阶段无解，问题就在哪一段

<!-- 旁白：左图是整个任务的阶段列表，右图放大了前几个阶段；调试 MTC 时先看哪个阶段解数为零，再回看该阶段的输入约束，这种可观测性是十步流程写法不具备的。 -->

---

## P14 · 实际案例：传送带抓取与码垛

- **要点：** 感知驱动的传送带抓取用"订阅位姿话题触发抓取"，码垛用位置数组逐层分配放置点

| 案例 | 输入/触发 | 关键实现 |
| --- | --- | --- |
| 传送带抓取 | 订阅 `/detected_object_pose` 话题 | approach(+0.12) → grasp → 闭爪 → retreat(+0.15) → place |
| 码垛应用 | `get_next_place_pose` 逐位分配 | `layer_positions` 数组，z = 0.15 + 层号×0.1 |

- 传送带抓取的时序要求：感知 → grasp 的平均延迟应小于传送带运动周期
- 官方感知管线入口：MoveIt Perceptions（`moveit_simple_grasps`）与 MTC 的 `GeneratePose` 阶段订阅自定义话题
- 建议：先用静态规划场景验证抓取位姿链，再接入真实感知

<!-- 旁白：两个案例代表抓放任务的两大变量——物体在动与目标在变：传送带考验感知与规划的时序配合，码垛考验放置策略的参数化；共同点是都复用本章的位姿链与 attach/detach 语义。 -->

---

## P15 · 本章要点

- **要点：**
- 抓取位姿链：approach → grasp → retreat，核心公式 `T_grasp = T_object × T_offset`
- 顶部抓取 (π, 0, 0) 朝下接近，侧面抓取沿 X/Y 水平接近，按物体形状选择
- 夹爪是独立规划组，开合即设置关节目标；抓取先开到略宽再闭合
- attach/detach 用 `AttachedCollisionObject` + `touch_links`，物体随末端运动
- Pick-and-Place 十步流程：抓取段与放置段对称，attach/detach 是分界点
- 官方三段式范式 + MTC 阶段化流水线：独立验证、并行解、失败回溯
- 感知驱动抓取：订阅物体位姿话题自动执行，注意感知延迟约束

<!-- 旁白：把本章收敛为一条主线：位姿链决定抓得住，夹爪与 attach/detach 决定拿得稳，十步流程与 MTC 决定做得规范，感知入口决定能否应对动态场景。 -->

---

## P16 · 练习题

1. 编写程序计算顶部抓取和侧面抓取的位姿，分析两种抓取方式适用的物体形状。

2. 实现Pick-and-Place完整流程：从A点抓取物体，移动到B点放置，最后回到home位姿。

3. 编写多物体分拣程序：3个不同位置的物体分别抓取并放置到3个对应的目标位置。

4. 实现码垛功能：将多个物体依次堆叠到指定位置，每层高度递增。

5. 编写程序模拟传送带抓取：订阅物体位姿话题，当检测到物体时自动执行抓取和放置。

<!-- 旁白：五道题由易到难：前两题练位姿链与完整流程，第三题练任务级路由，第四题练放置策略参数化，第五题练感知驱动，均可在 xArm 仿真环境验证。 -->

---

## P17 · 下章预告

- **要点：**
- 下一章进入第 30 章「ROS2 图像接口与相机标定」
- 先介绍 sensor_msgs 图像消息接口与相机信息消息
- 再讲相机内参、畸变系数与标定棋盘格的采集方法
- 最终完成相机标定流程并用标定参数验证图像处理效果

<!-- 旁白：抓取离不开对物体位置的感知，而感知的源头是相机：下一章进入图像世界，从图像消息接口讲到相机标定，为视觉伺服类任务打好基础。 -->
