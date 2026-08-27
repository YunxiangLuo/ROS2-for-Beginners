# 第1章 PPT：ROS 2 概述与架构设计

> 共 15 页，标注页码 · 图号与教学文档对应

---

## P1 · 标题页

**ROS 2 概述与架构设计**

- 课程：ROS2 Python 编程
- 章节：第 1 章
- 课时：2 课时

---

## P2 · 本课学习目标

- 理解 ROS 1 到 ROS 2 的演进动机
- 掌握 DDS 中间件架构与 RMW 抽象层
- 熟悉 QoS 服务质量策略
- 理解节点生命周期状态机
- 完成 ROS 2 环境搭建与 colcon 构建

---

## P3 · ROS 1 的三大局限

```
┌─────────┐     ┌──────────────┐     ┌──────────────┐
│ 单点故障 │     │  实时性不足   │     │  安全性缺失   │
│ Master   │     │ TCP 延迟不可控│     │ 无加密/认证   │
│ 崩溃 =   │     │ 无法硬实时    │     │ 无访问控制    │
│ 系统瘫痪 │     │              │     │              │
└─────────┘     └──────────────┘     └──────────────┘
       ↓  2014 年 Open Robotics 启动 ROS 2 项目  ↓
```

---

## P4 · ROS 1 vs ROS 2 架构对比

```
ROS 1 (中心化)                    ROS 2 (去中心化)
     ┌──────┐                    ┌─────┐  ┌─────┐
     │Master│                    │Node │  │Node │
     └─┬─┬──┘                    └──┬──┘  └──┬──┘
       │ │                          │  DDS  │
    ┌──┘ └──┐                       │ ────► │
    ▼       ▼                       ▼ 自动  ▼
 ┌────┐  ┌────┐                  ┌─────┐  ┌─────┐
 │Node│  │Node│                  │Node │  │Node │
 └────┘  └────┘                  └─────┘  └─────┘
    TCP 直连                        DDS RTPS
```

图 1-1：ROS 1 中心化 vs ROS 2 去中心化架构对比

---

## P5 · DDS 概念模型

| DDS 概念 | 对应 ROS 2 | 说明 |
|----------|-----------|------|
| DomainParticipant | Context | ROS 2 通信上下文 |
| DataWriter | Publisher | 消息发布者 |
| DataReader | Subscription | 消息订阅者 |
| Topic | Topic | 数据通道名称 |
| Domain | ROS_DOMAIN_ID | 通信域隔离 |

---

## P6 · RMW 分层架构

```
┌─────────────────────────┐
│   rclpy / rclcpp        │ 用户 API 层
├─────────────────────────┤
│   RCL (ROS Client Lib)  │ 客户端库层
├─────────────────────────┤
│   RMW (可替换中间件)      │ ← 关键抽象层
├─────────────────────────┤
│ Cyclone DDS / Fast DDS  │ DDS 实现层
├─────────────────────────┤
│   UDP / Shared Memory   │ 传输层
└─────────────────────────┘
```

图 1-2：ROS 2 分层架构，RMW 层实现 DDS 实现可替换

---

## P7 · 常用 QoS 策略维度

| 策略 | 选项 | 典型场景 |
|------|------|---------|
| Reliability | RELIABLE / BEST_EFFORT | 控制指令 / 传感器 |
| Durability | VOLATILE / TRANSIENT_LOCAL | 实时数据 / 配置 |
| History | KEEP_LAST(n) / KEEP_ALL | 限制内存 / 全量 |
| Depth | 正整数 | 队列缓冲大小 |
| Liveliness | AUTOMATIC | 发布者存活检测 |

---

## P8 · QoS 预定义配置

```python
# 传感器数据：高频、允许丢包
qos_sensor = qos_profile_sensor_data
# → BEST_EFFORT, KEEP_LAST(5)

# 服务通信：可靠传输
qos_service = qos_profile_services_default
# → RELIABLE, KEEP_LAST(10)
```

---

## P9 · 界面生命周期状态机

```
Unconfigured ──configure()──►  Inactive
    ▲                           │
    │                     activate() │
    │ cleanup()                ▼
    │                        Active
    │                           │
    └──── deactivate() ◄────────┘
```

图 1-3：生命周期节点五态转换

---

## P10 · 生命周期代码示例

```python
from rclpy.lifecycle import LifecycleNode

class MyNode(LifecycleNode):
    def on_configure(self, state): pass
    def on_activate(self, state):  pass
    def on_deactivate(self, state): pass
    def on_cleanup(self, state):   pass
```

---

## P11 · 安装总结 (Humble)

```bash
sudo apt install ros-humble-desktop-full
sudo apt install python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

---

## P12 · 工作空间结构

```
~/ros2_ws/
├── src/            ← 源码包目录
├── build/          ← 编译中间文件
├── install/        ← 安装输出
└── log/            ← 编译日志

$ colcon build --symlink-install
$ source install/setup.bash
```

---

## P13 · 本章要点

1. ROS 2 用 DDS 替换了 ROS 1 的 Master 中心化通信
2. RMW 层使 DDS 实现可替换（Fast DDS / Cyclone DDS）
3. QoS 提供细粒度的通信质量保证
4. 生命周期节点标准化了初始化激活流程
5. colcon 是 ROS 2 的标准构建工具

---

## P14 · 练习题

1. ROS 1 三个局限性及 ROS 2 解决方案？
2. ROS_DOMAIN_ID 作用？
3. RELIABLE + BEST_EFFORT 能否通信？
4. 创建 workspaces 并编译验证
5. 运行 talker/listener 体验

---

## P15 · 下章预告

**第 2 章：核心编程基础**

- 创建 Python 节点（rclpy）
- 日志系统
- rqt_graph / RViz2 可视化工具
