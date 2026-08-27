# 第16章 Nav2架构

---

## 学习目标
- 理解Nav2整体架构
- 掌握Nav2的核心组件和功能
- 了解生命周期管理机制
- 掌握Nav2的启动和配置方法

---

## Nav2简介
- ROS2官方导航框架
- Navigation2的简称
- ROS1 Navigation的升级版
- 模块化、插件化、可配置
- 支持多种机器人平台

---

## 系统架构概览
- 行为树驱动(BT Navigator)
- 全局规划器(Planner Server)
- 局部控制器(Controller Server)
- 代价地图(Costmap)
- 定位(AMCL)
- 行为恢复(Recovery)

---

## BT Navigator(行为树导航器)
- 核心决策引擎
- 使用行为树控制导航流程
- 可编辑XML行为树
- 支持条件节点、动作节点、控制节点
- 管理规划、控制、恢复行为

---

## Planner Server(规划器服务器)
- 提供全局路径规划
- 支持多种规划器插件
  - Navfn Planner (A*)
  - Smac Planner 2D/Hybrid
- 接收目标位姿, 输出全局路径
- 可配置规划器参数

---

## Controller Server(控制器服务器)
- 执行局部路径规划
- 输出速度指令
- 支持多种控制器插件
  - DWB (Dynamic Window Based)
  - TEB (Timed Elastic Band)
- 避障和轨迹跟踪

---

## Costmap(代价地图)
- 环境表示层
- 全局代价地图: 静态规划
- 局部代价地图: 动态避障
- 多层结构: 静态层、障碍物层、膨胀层
- 配置更新频率和范围

---

## AMCL定位模块
- 自适应蒙特卡洛定位
- 提供全局定位和位姿跟踪
- 发布/amcl_pose和/particlecloud
- 可与Cartographer纯定位模式切换
- 配置粒子数和观测模型

---

## 生命周期管理
- Nav2节点使用生命周期管理
- 状态: Unconfigured → Inactive → Active → Finalized
- 顺序启动确保组件就绪
- 安全的重启和关闭
- 故障检测和恢复

---

## 行为恢复(Recovery)
- 导航失败时的恢复策略
- Spin: 原地旋转
- Backup: 后退
- Wait: 等待
- Clear Costmap: 清除代价地图
- 可配置恢复序列

---

## Nav2数据流
1. 用户设置目标位姿
2. BT Navigator接收目标
3. Planner Server计算全局路径
4. Controller Server跟踪局部路径
5. Costmap更新障碍物信息
6. AMCL提供实时位姿
7. 到达目标或触发恢复

---

## Action通信
- NavigateToPose: 导航到目标点
- FollowWaypoints: 航点序列导航
- 基于ROS2 Action
- 支持进度反馈
- 可取消任务

---

## Nav2参数配置
- nav2_params.yaml 主配置文件
- 各组件参数独立命名空间
- 支持运行时动态重配置
- 参数继承和覆盖
- 常用工具: rqt_reconfigure

---

## 多机器人导航
- 每个机器人独立运行Nav2
- 共享地图
- 独立代价地图
- 协同避障
- 分布式任务分配

---

## Nav2性能优化
- 调整代价地图更新频率
- 优化规划算法参数
- 调整行为树刷新率
- 合理设置机器人速度
- 使用硬件加速

---

## 调试方法
- 可视化代价地图
- 监控生命周期状态
- 查看行为树状态
- 分析节点间通信延迟
- 使用Groot可视化行为树

---

## 实战配置要点
- 根据机器人特性调整速度限制
- 根据环境调整代价地图参数
- 配置合理的恢复行为
- 调整规划器适应场景
- 监控系统性能

---

## 总结
- Nav2是ROS2的完整导航解决方案
- 模块化架构方便定制和扩展
- 行为树驱动提供灵活的导航逻辑
- 生命周期管理保证系统可靠性
- 丰富的插件支持多种算法
- 适用于从教学到工业的各种场景
