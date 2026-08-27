# 第36章 自动驾驶概述与CARLA基础

---

## 学习目标
- 了解自动驾驶技术的分级标准与发展现状
- 掌握自动驾驶系统的三层架构
- 理解CARLA仿真平台的核心概念
- 学会搭建CARLA开发环境
- 掌握CARLA Python API的基本用法

---

## P1 · 标题页

**第36章：自动驾驶概述与CARLA基础**

ROS2 Python 编程课程

---

## P2 · 本章内容概览

1. 自动驾驶技术发展背景（36.1）
2. CARLA仿真平台介绍（36.2）
3. CARLA世界与地图系统（36.3）
4. 本章小结与练习题

---

## P3 · 自动驾驶分级 — SAE J3016

| 等级 | 名称 | 驾驶主体 | 监控主体 |
|:----:|------|:--------:|:--------:|
| L0 | 无自动化 | 人类 | 人类 |
| L1 | 驾驶辅助 | 人+系统 | 人类 |
| L2 | 部分自动化 | 系统 | 人类 |
| L3 | 有条件自动化 | 系统 | **系统** |
| L4 | 高度自动化 | 系统 | 系统 |
| L5 | 完全自动化 | 系统 | 系统 |

> L3是关键分水岭：责任从人类转移到系统

---

## P4 · 系统架构 — 感知→规划→控制

```
┌──────────────┐
│   感知层      │
│ 相机/LiDAR/Radar│
│ 多传感器融合   │
└──────┬───────┘
       ▼
┌──────────────┐
│   规划层      │
│ 行为决策/运动规划│
└──────┬───────┘
       ▼
┌──────────────┐
│   控制层      │
│ PID/MPC      │
└──────┬───────┘
       ▼
┌──────────────┐
│   执行机构    │
└──────────────┘
```

---

## P5 · 感知层详解

- **相机**：目标检测、车道线识别、交通信号灯
- **激光雷达**：3D点云、障碍物检测、SLAM
- **毫米波雷达**：远距离测速、全天候工作
- **超声波**：近距离泊车辅助
- **多传感器融合**：优势互补，提高鲁棒性

---

## P6 · 规划层详解

- **全局路径规划**：从起点到终点的道路级路径
  - A* / Dijkstra / Lattice Planner
- **行为决策**：换道、跟车、让行、停车
  - 有限状态机 / 行为树
- **运动规划**：生成无碰撞的轨迹
  - 路径规划 + 速度规划

---

## P7 · 控制层详解

- **纵向控制**：油门/刹车 → 速度控制
  - PID控制器：简单但需调参
  - MPC模型预测控制：考虑未来状态
- **横向控制**：方向盘 → 路径跟踪
  - Stanley方法：基于前轮偏差
  - Pure Pursuit：纯追踪算法

---

## P8 · CARLA仿真平台简介

- **C**ar **L**earning to **A**ct
- 基于 **Unreal Engine 4** 的开源自动驾驶仿真器
- 由巴塞罗那计算机视觉中心（CVC）研发
- **核心特性**：
  - 高保真渲染与物理引擎
  - 丰富的传感器套件（RGB、深度、语义、LIDAR...）
  - 内置Traffic Manager交通流控制
  - 完整Python API
  - ROS2桥接支持

---

## P9 · CARLA架构

```
┌──────────────────────┐
│    CARLA Server       │
│  ┌────┐ ┌──────────┐ │
│  │World│ │ Actor    │ │
│  │Map  │ │ Blueprint│ │
│  │Wx   │ │ Vehicle  │ │
│  │     │ │ Sensor   │ │
│  └────┘ └──────────┘ │
└────────┬─────────────┘
         │ TCP :2000
┌────────┴─────────────┐
│    CARLA Client       │
│  Python API           │
│  ROS2 Bridge          │
└──────────────────────┘
```

---

## P10 · 核心概念对照

| 概念 | 类比 | Python类 |
|------|------|----------|
| World | 仿真世界实例 | `carla.World` |
| Map | 地图+路网 | `carla.Map` |
| Actor | 动态实体 | `carla.Actor` |
| Blueprint | 工厂模板 | `carla.ActorBlueprint` |
| Sensor | 数据采集器 | `carla.Sensor` |
| Traffic Mgr | 交通警察 | `carla.TrafficManager` |

---

## P11 · Python API 快速入门

```python
import carla

# 连接服务器
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# 获取蓝图
bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.find('vehicle.tesla.model3')

# 生成车辆
spawn_pt = world.get_map().get_spawn_points()[0]
vehicle = world.spawn_actor(vehicle_bp, spawn_pt)

# 控制车辆
vehicle.apply_control(
    carla.VehicleControl(throttle=0.5, steer=0.0)
)
```

---

## P12 · Town地图一览

| 地图 | 特点 | 复杂度 |
|:----:|------|:------:|
| Town01 | 简单直线道路 | ★ |
| Town02 | 小规模T型路口 | ★★ |
| Town03 | 环岛+立交 | ★★★ |
| Town04 | 大型高速路 | ★★★ |
| Town05 | 城市+高速混合 | ★★★★ |
| Town10 | 大城市天际线 | ★★★★★ |

> 入门推荐：Town01 → Town03 → Town05

---

## P13 · 天气系统

```python
# 预设天气
world.set_weather(carla.WeatherParameters.ClearNoon)
world.set_weather(carla.WeatherParameters.HardRainNoon)
world.set_weather(carla.WeatherParameters.WetCloudySunset)

# 自定义天气
weather = carla.WeatherParameters(
    cloudiness=80.0,       # 云量
    precipitation=60.0,    # 降雨
    fog_density=20.0,      # 雾浓度
    sun_altitude_angle=30.0, # 太阳高度角
)
```

云量/降水/积水/风速/雾/太阳角度 → 全可控

---

## P14 · 传感器配置

```python
# RGB相机
camera_bp = bp_lib.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '1280')
camera_bp.set_attribute('image_size_y', '720')
camera_bp.set_attribute('fov', '110')

# 激光雷达
lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('channels', '64')
lidar_bp.set_attribute('range', '100')
lidar_bp.set_attribute('points_per_second', '1000000')

# 语义分割相机
semantic_bp = bp_lib.find('sensor.camera.semantic_segmentation')
```

---

## P15 · 总结与思考

**核心要点：**
- SAE L0-L5定义了自动驾驶的六个等级
- 感知→规划→控制是经典系统架构
- CARLA是功能最完善的开源自动驾驶仿真器
- Python API提供了完整的仿真控制能力

**思考题：**
1. L3级自动驾驶面临哪些技术挑战和法律问题？
2. CARLA仿真中的仿真-真实差距（Sim2Real）如何缩小？
3. 如何在CARLA中评估一个自动驾驶算法的性能？
