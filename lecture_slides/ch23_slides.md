# 第23章 PPT：SLAM与导航综合实训

> 共 16 页，标注页码 · 图号与教学文档对应

---

## P1 · 标题页

- 课程：ROS2 Python 编程
- 章节：第 23 章 SLAM 与导航综合实训
- 课时：4 课时
- 内容：完整建图流程、Nav2 导航、多机器人调度、项目文档与性能评估

<!-- 旁白：欢迎来到第 23 章！本次课共 4 课时，是 SLAM 与导航的综合实训。前面十几章的激光建图、定位、导航知识要在这一章全部串起来：完整建图流程、Nav2 导航、多机器人调度和项目文档。这一章以动手为主，每一条命令都值得亲自敲一遍。 -->

---

## P2 · 学习目标

- 掌握从 SLAM 建图到自主导航的完整流程
- 能够在仿真环境中完成建图、保存地图和导航
- 理解多机器人调度系统的设计方法
- 培养项目文档编写和方案设计能力
- 掌握 SLAM 与导航系统集成的工程实践

<!-- 旁白：五条目标都是"能不能做出来"型的：完整流程、仿真建图导航、调度系统设计、文档能力、系统集成，核心是打通「建图→保存→定位→导航」这条链路。特别提醒：文档能力这条最容易被轻视，但它决定了项目最后能不能交接、能不能复用。 -->

---

## P3 · 实训概述

- 实训目标：打通「建图 → 保存 → 定位 → 导航」全链路
- 实训要求：
  - 使用课程 Wheeltec 仿真环境完成全部实验
  - 输出可复现的配置文件与启动脚本
  - 按模板撰写完整项目设计文档
- 实训产出：地图文件、导航配置、调度系统设计、评估报告

<!-- 旁白：实训要求三条线：全流程在 Wheeltec 仿真环境完成、输出可复现的配置与启动脚本、按模板写设计文档。最终产出四样东西——地图文件、导航配置、调度系统设计、评估报告。注意"可复现"三个字：任何配置和脚本都要做到别人拿到就能跑。 -->

---

## P4 · slam_toolbox 在线建图

- 完整建图命令流：

```
# 启动课程 Wheeltec 仿真环境
ros2 launch robot_sim_demo gazebo2.launch.py drive:=false

# 启动 slam_toolbox 在线建图
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=./config/mapper_params_online_async.yaml \
  use_sim_time:=true

# 控制机器人探索
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 保存地图
ros2 run nav2_map_server map_saver_cli -f ~/maps/final_map
```

- 在线异步建图支持边走边建，探索结束后用 map_saver_cli 保存 PGM + YAML

![图 23-w1：slam_toolbox 在线同步建图](images/web/ch10/slam_toolbox_sync.png)

<!-- 旁白：这张图是 slam_toolbox 在线建图的效果，地图边探索边扩展。四条命令一条条看：先起仿真环境，再启 online_async 异步建图，用 teleop 控制机器人探索，最后 map_saver_cli 保存 PGM 加 YAML。在线异步建图的核心优势就是边走边建、随时可查，导航实验的救命稻草。 -->

---

## P5 · 自动探索建图：AutoExplorer

- 节点结构：发布 /cmd_vel，订阅 /scan 与 /map
- 状态机：explore / rotate / forward 三态切换
- 控制参数：max_speed=0.22、max_rotation=0.8、safe_distance=0.5，控制周期 0.1 s
- 障碍策略：激光前 / 左 / 右三向距离判断，前方过近则旋转避让
- 覆盖率 > 95% 时自动停止探索并触发保存地图

<!-- 旁白：自动探索本质上是个小状态机：explore、rotate、forward 三态循环。max_speed 0.22、max_rotation 0.8、safe_distance 0.5 三个参数控制探索行为，激光测前、左、右三向距离，前方过近就旋转避让。覆盖率超过 95% 自动停止并保存地图，这就是"探索完成"的客观标准。 -->

---

## P6 · Nav2 导航配置：规划与控制

- 规划器 planner_server：NavfnPlanner，tolerance=0.5，use_astar=true
- 控制器 controller_server：DWB 局部规划器

| 参数 | 数值 |
| --- | --- |
| max_vel_x | 0.5 |
| max_vel_theta | 1.0 |
| acc_lim_x | 2.5 |
| acc_lim_theta | 3.2 |

- DWB 评分器：RotateToGoal / Oscillation / BaseObstacle / GoalAlign / PathAlign / PathDist / GoalDist（RotateToGoal.scale=32.0、GoalDist.scale=24.0）
- 行为树：default_bt_xml_filename = navigate_to_pose.xml，含 spin / backup / wait / clear_costmap 等节点

![图 23-w2：Nav2 系统架构](images/web/ch05/nav2_architecture.png)

<!-- 旁白：这张架构图把 Nav2 的分层说清楚了：规划器、控制器、行为树三大件。planner_server 用 NavfnPlanner，控制器用 DWB，评分器里面 RotateToGoal 权重 32、GoalDist 24 最高。行为树 navigate_to_pose.xml 里的 spin、backup、wait、clear_costmap 都是恢复行为——导航僵住的时候靠它们自救。 -->

---

## P7 · 代价地图与 AMCL

- local_costmap：5 Hz 更新，odom 坐标系，滚动窗口 4×4 m，分辨率 0.05，voxel_layer + inflation_layer
- global_costmap：1 Hz 更新，map 坐标系，static + obstacle + inflation 三层，track_unknown_space=true
- 膨胀参数：inflation_radius=0.55（约机器人半径 ×2），cost_scaling_factor=3.0
- AMCL 定位：

| 参数 | 数值 |
| --- | --- |
| max_particles / min_particles | 2000 / 500 |
| laser_model_type | likelihood_field |
| update_min_d / update_min_a | 0.1 / 0.2 |
| z_hit / z_rand | 0.95 / 0.05 |

<!-- 旁白：代价地图分本地和全局两本账：局部 5 赫兹滚动窗口 4×4 米，全局 1 赫兹挂在 map 坐标系。膨胀半径 0.55 米约等于机器人半径两倍，cost_scaling_factor 取 3.0。AMCL 表格里记四个关键词：2000 对 500 的粒子数、likelihood_field 激光模型、0.95 对 0.05 的 z_hit 与 z_rand。 -->

---

## P8 · 导航启动与多点导航

- bringup_navigation.py：生命周期节点自动激活 + AMCL 初始位姿注入
- NavigationConsole 封装 BasicNavigator：

```
navigator.goToPose(pose)      # 发送单点导航目标
navigator.waitUntilNav2Active()  # 等待 Nav2 激活
```

- 多点导航控制台：依次发送航点、读取反馈状态、支持取消与重试
- 仿真实例：从在线建图切换到 Nav2 导航，验证 map → odom → base_link TF 链

<!-- 旁白：从建图切到导航，靠 bringup 脚本把生命周期节点自动激活、注入 AMCL 初始位姿。NavigationConsole 封装得更顺手：goToPose 发目标、waitUntilNav2Active 等系统就绪。多点导航就是依次发航点、读反馈、支持取消与重试。别忘了验证 map 到 odom 再到 base_link 这条 TF 链。 -->

---

## P9 · 多机器人调度系统

- 调度架构三层：

| 层级 | 职责 |
| --- | --- |
| 任务管理层 | 任务分配、优先级排序 |
| 路径协调层 | 航线规划、冲突避免 |
| 机器人执行层 | 单机导航、状态上报 |

- 冲突避免：区域锁定 + 会车点等待 + 优先级仲裁
- 状态监控：各机器人位姿、电量、任务进度统一上报
- 注意：本仓库仅提供单台 Wheeltec 仿真入口，多机调度以设计验证为主

<!-- 旁白：调度系统分三层：任务管理层做分配和优先级排序，路径协调层做航线和冲突避免，机器人执行层做单机导航和状态上报。冲突避免三件套——区域锁定、会车点等待、优先级仲裁。诚实提醒：仓库里只有单台 Wheeltec 仿真入口，多机调度以设计验证为主。 -->

---

## P10 · 项目文档规范

- 文档结构：需求分析 → 系统架构 → 参数配置 → 测试方案 → 问题记录
- 设计文档模板：项目名称「SLAM 与导航综合实训项目」
- 测试方案要素：测试环境、测试用例、通过标准、回归方法
- 问题记录表：现象、原因分析、解决措施、验证结果
- 文档与代码同版本管理，配置变更须同步更新

<!-- 旁白：文档规范五大节：需求分析、系统架构、参数配置、测试方案、问题记录。测试方案必须写清测试环境、测试用例、通过标准和回归方法；问题记录表用"现象—原因—措施—验证"四列。记住一条铁律：文档与代码同版本管理，配置变更必须同步更新。 -->

---

## P11 · 建图质量评估：MapEvaluator

| 指标 | 计算方法 | 含义 |
| --- | --- | --- |
| coverage | 非灰色（≠205）像素占比 | 探索覆盖程度 |
| accuracy | 与真值 absdiff>50 像素的反比 | 与真值一致性 |
| clarity | Canny 边缘密度 | 地图边缘锐度 |
| resolution | 0.05 m/pixel | 地图分辨率 |

- 评估报告模板：成功率 93% 等指标 + 各项数值 + 结论建议
- 有真值时优先算 accuracy，无真值时以 coverage + clarity 为准

<!-- 旁白：这张指标表是客观评价地图的尺子：coverage 看非灰色像素占比，accuracy 和真值做差、clarity 用 Canny 边缘密度，分辨率固定 0.05 米每像素。评估口诀：有真值算 accuracy，没真值用 coverage 加 clarity。报告里写清"成功率 93%"这类指标再加结论建议才算完整。 -->

---

## P12 · 导航性能优化清单

- 建图阶段：充分探索（覆盖率 >95%）、后处理滤波、设置地图原点
- 定位阶段：正确初始位姿、AMCL 粒子数 500–2000、激光模型选择、TF 检查
- 规划阶段：全局规划器选型、膨胀半径 = 机器人半径 ×2、代价地图更新频率、规划超时
- 控制阶段：最大速度、DWB 评分器权重、恢复行为、前瞻距离（Pure Pursuit）
- 系统级：大型环境降低代价地图分辨率、增大局部地图、生命周期节点自动启动

<!-- 旁白：优化清单按四大阶段排：建图阶段要覆盖率超 95%、后处理滤波、设好地图原点；定位阶段管好初始位姿和粒子数；规划阶段看膨胀半径与代价地图更新频率；控制阶段调速度、评分器权重和恢复行为。系统级记住两招——大环境降地图分辨率、局部地图加大。 -->

---

## P13 · 常见问题排查

- 建图漂移：检查激光频率（>5 Hz）、降低移动速度、启用回环检测
- 地图错位：检查里程计精度、增加粒子数、调整扫描匹配参数
- 地图不完整：增加探索时间、检查激光范围设置、确保死角覆盖
- 导航失败排查顺序：TF 链 → 代价地图 → 规划器 → 控制器 → 恢复行为
- 排查工具：rviz 可视化代价地图、ros2 topic echo 查看目标状态

<!-- 旁白：排查按现象找因：建图漂移查激光频率和移动速度，地图错位查里程计和粒子数，地图不完整就加探索时间、查激光范围。导航失败有固定排查顺序——TF 链、代价地图、规划器、控制器、恢复行为，照这个顺序查最省时间，rviz 看代价地图、topic echo 看目标状态。 -->

---

## P14 · 本章要点

- 完整流程：slam_toolbox 建图 → map_saver_cli 保存 → AMCL 定位 → Nav2 导航
- Nav2 三大件：规划器（NavfnPlanner）、控制器（DWB）、行为树（bt_navigator）
- 代价地图分层：static + obstacle + inflation，膨胀半径约机器人半径 ×2
- AutoExplorer 状态机自动探索，覆盖率 >95% 自动保存地图
- 调度系统三层架构：任务管理、路径协调、机器人执行
- 量化评估驱动优化：coverage / accuracy / clarity 三指标

<!-- 旁白：六条要点就是全章的审计清单：完整流程、Nav2 三大件、代价地图分层、AutoExplorer 状态机、调度三层架构、三项评估指标。能把这六条各自展开讲清楚，你的实训就不再是"跑通了"，而是真正内化了。 -->

---

## P15 · 课后练习

1. 操作题：完成完整流程：启动仿真 → slam_toolbox 建图 → 保存地图 → AMCL 定位 → 发送导航目标
2. 编程题：实现自动探索建图节点，探索覆盖率超过 95% 时自动保存地图
3. 设计题：设计 3 台机器人的调度系统（任务分配、冲突避免、状态监控），给出架构图与核心代码框架
4. 分析题：分析流程中哪些环节容易引入误差，以及如何通过参数调优减小误差
5. 文档题：仿照设计文档模板撰写完整设计文档（需求分析、系统架构、参数配置、测试方案、问题记录）
6. 综合题：为大型商超设计机器人服务方案（清洁、巡检、导购），覆盖跨楼层 SLAM、任务优先级调度、人机交互与系统健康管理

<!-- 旁白：六道题正好对应实训的六个侧面：第一题完整流程走一遍，第二题写自动探索节点，第三题设计三机调度系统，第四题分析误差来源与调优，第五题写设计文档，第六题大商超综合方案。第三题和第五题最值得认真做——调度架构图和设计文档是作品集里最拿得出手的东西。 -->

---

## P16 · 下章预告

- 第 24 章：机械臂基础知识
- 内容：机械臂分类、运动学与 DH 参数、正逆运动学、工作空间

<!-- 旁白：下一章换战场：从移动机器人进入机械臂！第 24 章讲机械臂基础知识——分类、运动学、DH 参数、正逆解和工作空间。提前把线性代数的记忆找回来，我们下节课见！ -->