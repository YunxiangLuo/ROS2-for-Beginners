# 第26章 MoveIt2基础

---

## 学习目标
- 掌握MoveIt2的架构和组件
- 学会使用Setup Assistant配置机械臂
- 理解运动规划的基本流程
- 能够在RViz2中使用MotionPlanning插件

---

## 什么是MoveIt2?
- ROS2中最流行的运动规划框架
- 提供: IK/FK求解, 碰撞检测, 路径规划
- 支持: 多种规划器(OMPL, Pilz)
- 应用: 工业机械臂, 移动操作臂

---

## MoveIt2架构
- move_group: 中心节点
- Planning Scene: 规划场景管理
- Robot Model: 机器人模型
- Planning Pipeline: 规划管道
- Trajectory Execution: 轨迹执行

---

## Setup Assistant功能
- 加载URDF模型
- 生成自碰撞矩阵
- 配置规划组
- 添加预设位姿
- 配置末端执行器
- 生成配置功能包

---

## 自碰撞矩阵
- 检测哪些link对不可能碰撞
- 减少碰撞检测计算量
- 采样密度: 默认10000
- 生成后需人工确认

---

## 规划组(Planning Group)
- 一组joint和link的集合
- 每个组独立进行规划
- 示例:
  - xarm_group: arm_1~6_joint
  - gripper_group: gripper_1~2_joint
- 每个组可独立配置求解器

---

## 运动学求解器
- KDL: 默认, 基于数值迭代
- TRAC-IK: 更鲁棒, 速度更快
- 选择依据: 机械臂构型, 精度要求
- 可在配置中切换

---

## 末端执行器(End Effector)
- 定义在规划组末端
- 用于IK目标指定
- 支持附着的物体
- 典型: gripper, welding_torch, camera

---

## MotionPlanning插件
- RViz2中的核心插件
- Start State: 起始状态(绿色)
- Goal State: 目标状态(橙色)
- Plan: 规划路径
- Execute: 执行轨迹
- Scene Objects: 添加障碍物

---

## 规划流程
1. 设置起始状态
2. 设置目标状态(关节/位姿)
3. 调用规划器
4. 检查碰撞
5. 生成轨迹
6. 执行轨迹

---

## 正运动学规划
- 通过滑块拖动关节
- 直接设置关节目标值
- 不需要IK求解
- 适合简单位置调整

---

## 逆运动学规划
- 设置末端位姿目标
- 拖动球/环交互调整
- IK求解器自动计算关节角
- 适合精确位姿控制

---

## 预设位姿
- Home: 初始位姿
- vertical: 垂直位姿
- random valid: 随机有效位姿
- 用户自定义: 常用工作位姿

---

## 规划场景(Planning Scene)
- 机器人自身模型
- 障碍物(CollisionObject)
- 附着物体(AttachedCollisionObject)
- 颜色和透明度设置

---

## OMPL规划器
- Open Motion Planning Library
- 多种规划算法:
  - RRT: 快速随机扩展树
  - RRTConnect: 双向RRT
  - PRM: 概率路标图
  - LazyPRM: 延迟碰撞检测

---

## 碰撞检测
- FCL (Flexible Collision Library)
- 自碰撞: 机械臂自身link间
- 环境碰撞: 与障碍物
- 分为: 离散/连续碰撞检测
- 影响规划速度的关键因素

---

## 轨迹执行
- 规划结果: RobotTrajectory
- 包含: JointTrajectory + 执行时间
- follow_joint_trajectory Action
- ros2_control: 硬件接口

---

## MoveIt2命令行
```bash
ros2 run moveit_commander moveit_commander_cmdline.py
```
- use: 选择规划组
- current: 查看当前状态
- rec/go: 记录/执行位姿
- plan/execute: 规划执行

---

## 思考
- 为什么需要自碰撞矩阵?
- 规划器选择对路径质量有何影响?
- 如何解决规划失败的问题?
- 规划时间对实时性的影响?

---

## 总结
- MoveIt2是ROS2机械臂编程的核心框架
- Setup Assistant简化了配置流程
- MotionPlanning插件提供交互式规划
- 碰撞检测确保运动安全
- 支持多种规划器适应不同场景
