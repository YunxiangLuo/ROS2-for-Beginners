# 第1章 PPT：ROS 2 概述与架构设计

> 共 16 页，标注页码 · 图号与教学文档对应

---

## P1 · 标题页

**ROS 2 概述与架构设计**

- 课程：ROS2 Python 编程
- 章节：第 1 章
- 课时：2 课时

---

## P2 · 本课学习目标

- 理解 ROS 1 的局限性及 ROS 2 的重新设计动机
- 掌握 DDS 去中心化架构与自动发现机制
- 理解 RMW 抽象层与 DDS 实现可替换性
- 掌握 QoS 核心策略维度与预定义配置
- 理解生命周期节点状态机
- 完成 ROS 2 安装、工作空间创建与 colcon 构建

---

## P3 · ROS 1 的四大局限

```
┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐
│ 单点故障 │ │ 实时性   │ │ 安全性   │ │ 嵌入式   │
│ Master   │ │ 不足     │ │ 缺失     │ │ 支持薄弱 │
│ 崩溃 =   │ │ TCP 延迟 │ │ 无认证   │ │ 微控制器 │
│ 系统瘫痪 │ │ 不可控   │ │ 授权加密 │ │ 资源受限 │
└──────────┘ └──────────┘ └──────────┘ └──────────┘
        ↓ 2014 年 Open Robotics 启动 ROS 2 ↓
```

- 2007 年 Willow Garage 发布 ROS 1，2014 年 ROS 2 完全重新设计通信架构

---

## P4 · DDS 去中心化架构

```
图 1-1：DDS 去中心化架构
┌─────────────┐                    ┌─────────────┐
│   Node A    │                    │   Node B    │
│             │                    │             │
│ ┌─────────┐ │    DDS Domain     │ ┌─────────┐ │
│ │Publisher│ │◄.`.`Topic.`.`.`.►│ │Subscribe│ │
│ └─────────┘ │    (自动发现)      │ └─────────┘ │
└─────────────┘                    └─────────────┘
        │                                  │
        └────────── 去中心化 ─────────────┘
               (无 Master 节点)
```

- 节点通过 DDS 自动发现机制直接对等通信

---

## P5 · 版本演进与选型

| 版本 | 发布日期 | 状态 |
|------|---------|------|
| Foxy | 2020.06 | EOL |
| Galactic | 2021.05 | EOL |
| **Humble** | 2022.05 | **LTS (推荐)** |
| Iron | 2023.05 | EOL |
| **Jazzy** | 2024.05 | **LTS (最新)** |

- 本课程基于 ROS 2 Humble (Ubuntu 22.04) 或 Jazzy (Ubuntu 24.04)

---

## P6 · DDS 概念模型

| DDS 概念 | 对应 ROS 2 | 说明 |
|----------|-----------|------|
| DomainParticipant | Node (通过 context) | 通信参与者 |
| DataWriter | Publisher | 消息发布者 |
| DataReader | Subscription | 消息订阅者 |
| Topic | Topic | 数据通道名称 |
| Domain | ROS_DOMAIN_ID | 通信域隔离 |

- DDS 是 OMG 制定的分布式实时通信标准，底层使用 RTPS 协议

---

## P7 · RMW 分层架构

```
┌──────────────────┐
│  rclcpp / rclpy  │  ← 用户代码 API
├──────────────────┤
│       RCL        │  ← ROS Client Library
├──────────────────┤
│       RMW        │  ← 中间件抽象层 (可替换)
├──────────────────┤
│  Fast DDS /      │
│  Cyclone DDS /   │  ← DDS 实现 (可切换)
│  RTI Connext     │
├──────────────────┤
│   UDP / SHMEM    │  ← 传输层
└──────────────────┘
```

图 1-2：ROS 2 分层架构

- RMW 层允许在不修改应用代码的情况下切换 DDS 实现（Fast DDS / Cyclone DDS / RTI Connext）
- 三大收益：多机分布部署、QoS 内建于通信层、符合实时通信标准

---

## P8 · QoS 策略维度

| 策略维度 | 选项 | 含义 |
|---------|------|------|
| Reliability | RELIABLE / BEST_EFFORT | 可靠传输 / 尽力传输 |
| Durability | VOLATILE / TRANSIENT_LOCAL | 不保存 / 保存历史 |
| History | KEEP_LAST(n) / KEEP_ALL | 保留最后 n 条 / 保留所有 |
| Depth | 正整数 | 队列深度 |
| Deadline | 时间间隔 | 消息更新最大间隔 |
| Lifespan | 时间间隔 | 消息有效期 |
| Liveliness | AUTOMATIC / MANUAL | 发布者存活检测 |

---

## P9 · QoS 预定义配置

```python
from rclpy.qos import QoSProfile, qos_profile_sensor_data, \
    qos_profile_services_default

# 传感器数据：高频，允许丢包
qos_sensor = qos_profile_sensor_data  # BEST_EFFORT, KEEP_LAST(5)

# 服务通信：可靠传输
qos_service = qos_profile_services_default  # RELIABLE, KEEP_LAST(10)

# 自定义 QoS
qos_custom = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10)
```

---

## P10 · 节点生命周期状态机

```
图 1-3：生命周期节点状态转换
                            configure()
Unconfigured ─────────────────────► Inactive
     ▲                                  │
     │                           activate()
     │ cleanup()                        ▼
     │                                Active
     │                                  │
     └──────── deactivate() ───────────┘

     shutdown() (从任意状态进入 Finalized)
```

- 通过标准状态机管理初始化与关停，保证启动/退出行为确定可控

---

## P11 · 生命周期 Python API

```python
import rclpy
from rclpy.lifecycle import LifecycleNode, State, \
    TransitionCallbackReturn

class MyLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('my_lifecycle_node')

    def on_configure(self, state: State):
        # 分配资源、设置参数
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        # 开始处理数据
        return TransitionCallbackReturn.SUCCESS
```

---

## P12 · 安装与环境搭建 (Humble)

```bash
# 安装 ROS 2 Humble 完整版
sudo apt install ros-humble-desktop-full

# 安装开发工具
sudo apt install python3-colcon-common-extensions \
    python3-rosdep python3-vcstool

# 环境变量写入 ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

- rosdep 用于解析并安装包依赖

---

## P13 · 工作空间与 colcon 构建

```
~/ros2_ws/
├── src/       ← 源码包目录
├── build/     ← 编译中间文件
├── install/   ← 安装输出
└── log/       ← 编译日志
```

```bash
colcon build --symlink-install   # Python 包免重编
source install/setup.bash        # 每次新终端都要 source
ros2 run demo_nodes_py listener  # 验证安装
```

---

## P14 · 本章要点

1. ROS 2 用 DDS 去中心化架构替换 ROS 1 的 Master 中心化通信
2. RMW 抽象层使 DDS 实现可替换（Fast DDS / Cyclone DDS）
3. QoS 从可靠性、持久性、历史、深度等维度提供细粒度保障
4. 生命周期节点用标准状态机管理配置、激活与关闭流程
5. colcon 是 ROS 2 标准构建工具，工作空间含 src/build/install/log
6. 新终端必须重新 source，否则报 Package not found

---

## P15 · 练习题

1. 简述 ROS 1 的三个主要局限性及 ROS 2 对应解决方案
2. ROS_DOMAIN_ID 的作用？两台机器域 ID 不同能否通信？
3. Publisher: RELIABLE, VOLATILE 与 Subscriber: BEST_EFFORT, VOLATILE 能否匹配？
4. 查看 ROS_DOMAIN_ID 并修改为 42 验证
5. 创建工作空间，colcon build 编译并验证 setup.bash 加载
6. 运行 talker/listener，用 ros2 node list 和 ros2 topic list 观察

> 提示：练习 3 关注 QoS 匹配规则——订阅者要求不得高于发布者能力

---

## P16 · 下章预告

**第 2 章：核心编程基础**

- 创建第一个 Python 节点（rclpy）
- 日志系统与参数使用
- rqt_graph / RViz2 可视化工具
