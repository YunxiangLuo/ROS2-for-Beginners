# 第43章 行为决策与交通规则

## 幻灯片目录

---

## 幻灯片 1: 封面

```
╔═══════════════════════════════════════════════╗
║      第43章 行为决策与交通规则               ║
║                                              ║
║    ROS2 × CARLA 自动驾驶编程技术课程          ║
║    43.1 行为决策概述                         ║
║    43.2 有限状态机 (FSM)                     ║
║    43.3 交通灯识别与响应                     ║
║    43.4 Traffic Manager                      ║
║    43.5 Scenario Runner                      ║
╚═══════════════════════════════════════════════╝
```

---

## 幻灯片 2: 课程大纲

```
┌─ 43.1 行为决策概述 ─────────────────────────┐
│  • 分层决策架构                              │
│  • 行为规划 vs 运动规划                      │
│  • 关键输入与输出                            │
├─ 43.2 有限状态机 ───────────────────────────┤
│  • 状态定义 (巡航/跟车/变道/停车/避障)      │
│  • 状态转移图与转移表                        │
├─ 43.3 交通灯识别与响应 ─────────────────────┤
│  • CARLA 交通灯 API                          │
│  • 灯色分类与停止线判断                      │
├─ 43.4 Traffic Manager ──────────────────────┤
│  • 内置交通管理模块                          │
│  • 参数配置与驾驶风格                        │
├─ 43.5 Scenario Runner ──────────────────────┤
│  • 场景测试框架                              │
│  • 自定义场景开发                            │
└──────────────────────────────────────────────┘
```

---

## 幻灯片 3: 分层决策架构

```
      ╔═══════════════════════════════╗
      ║    任务规划 (Route Planning)  ║
      ║    全局路径: A → B            ║
      ╚═══════╦═══════════════════════╝
              ↓ 全局路径
      ╔═══════════════════════════════╗
      ║  ★ 行为决策 (行为规划)       ║  ← 本章重点
      ║    FSM / Behavior Tree        ║
      ╚═══════╦═══════════════════════╝
              ↓ 目标行为 + 目标速度
      ╔═══════════════════════════════╗
      ║    运动规划 (Motion Planning) ║
      ║    轨迹生成 (路径+速度曲线)   ║
      ╚═══════╦═══════════════════════╝
              ↓ 轨迹点
      ╔═══════════════════════════════╗
      ║    控制 (Control)             ║
      ║    PID / MPC                  ║
      ╚═══════════════════════════════╝
```

**关键点**: 行为决策将任务规划的宏观路径，转化为具体的驾驶操作指令。

---

## 幻灯片 4: FSM 状态定义

| 状态 | 图标 | 含义 |
|------|------|------|
| CRUISE | 🏁 | 车道保持，匀速巡航 |
| FOLLOW | 🚗→🚗 | 跟车，保持安全距离 |
| CHANGE_LEFT | ⬅️ | 向左变道 |
| CHANGE_RIGHT | ➡️ | 向右变道 |
| STOP | 🛑 | 停止（红灯/行人） |
| AVOID | ⚠️ | 紧急避障 |
| COMPLETE | ✅ | 到达目的地 |

**设计原则**:
- 状态应该互斥且完备
- 每个状态有明确的进入/退出条件
- 状态数量控制在 5~8 个为宜

---

## 幻灯片 5: FSM 状态转移图

```
                         ┌──────────┐
         ┌───────────────┤  CRUISE  ├───────────┐
         │               └────┬─────┘           │
    前车接近            绿灯/无障碍         前车慢
         │                  │                  │
         v                  v                  v
    ┌────────┐       ┌──────────┐     ┌─────────────┐
    │ FOLLOW │◄──────┤   STOP   ├────►│ CHANGE_LEFT │
    └───┬────┘       └──────────┘     └──────┬──────┘
        │                  ▲                  │
   前车消失         红灯/行人            变道完成
        │                  │                  │
        └─────────┬────────┘                  │
                  v                           v
              ┌────────┐             ┌─────────────┐
              │ CRUISE │             │ CHANGE_RIGHT│
              └────────┘             └─────────────┘
```

---

## 幻灯片 6: FSM Python 实现框架

```python
class FsmState(Enum):
    CRUISE       = 1
    FOLLOW       = 2
    CHANGE_LEFT  = 3
    CHANGE_RIGHT = 4
    STOP         = 5
    AVOID        = 6

class BehaviorFSM:
    def __init__(self):
        self.state = FsmState.CRUISE
        self.handlers = {
            FsmState.CRUISE:       self._handle_cruise,
            FsmState.FOLLOW:       self._handle_follow,
            FsmState.STOP:         self._handle_stop,
            FsmState.CHANGE_LEFT:  self._handle_change_left,
            FsmState.CHANGE_RIGHT: self._handle_change_right,
            FsmState.AVOID:        self._handle_avoid,
        }

    def update(self, perception):
        handler = self.handlers[self.state]
        new_state, cmd = handler(perception)
        self.state = new_state
        return cmd
```

**执行流程**: 当前状态 → 处理函数 → 条件判断 → 转移/保持 → 输出指令

---

## 幻灯片 7: 交通灯状态机

```
                  ┌─────────┐
                  │ UNKNOWN │  ← 初始化/未检测到
                  └────┬────┘
                       │ 检测到交通灯
                       v
                  ┌─────────────┐
                  │ APPROACHING │  ← 接近路口，判断灯色
                  └──────┬──────┘
                         │
              ┌──────────┼──────────┐
              │          │          │
              v          v          v
          ┌──────┐  ┌────────┐  ┌───────┐
          │ RED  │  │ YELLOW │  │ GREEN │
          └──┬───┘  └───┬────┘  └───┬───┘
             │          │           │
          红灯亮    黄灯>3s      绿灯亮
             │          │           │
             v          v           v
         ┌───────┐ ┌────────┐ ┌─────────┐
         │ STOP  │ │ 减速   │ │ CRUISE  │
         │ 停车  │ │ 准备停 │ │ 匀速通过│
         └───────┘ └────────┘ └─────────┘
```

---

## 幻灯片 8: CARLA 交通灯 API

```python
import carla

# 连接 CARLA
client = carla.Client('localhost', 2000)
world = client.get_world()

# 获取自车影响的下一个交通灯
traffic_light = vehicle.get_traffic_light()

if traffic_light is not None:
    state = traffic_light.get_state()
    # state → Red / Yellow / Green / Off

    # 获取停止线位置
    stop_wps = traffic_light.get_stop_waypoints()
    stop_location = stop_wps[0].transform.location

    # 计算到停止线的距离
    dist = vehicle.get_location().distance(stop_location)

    # 判断是否需要停止
    if state == carla.TrafficLightState.Red and dist < 15.0:
        print("🛑 红灯，准备停止")
    elif state == carla.TrafficLightState.Green:
        print("✅ 绿灯，正常通过")
```

---

## 幻灯片 9: 停止线判断逻辑

```
车辆位置 ●
   │
   │ 距离 d
   │
停止线 ──────────────
   │
   │
交通灯 🔴🟡🟢
```

**判断规则**:

| 灯色 | 距离 d < 3m | 3m < d < 15m | d > 15m |
|------|:-----------:|:------------:|:-------:|
| 🔴 红 | 强制停止 | 减速停止 | 准备停止 |
| 🟡 黄 | 减速停止 | 判断时间 | 正常行驶 |
| 🟢 绿 | 正常通过 | 正常通过 | 正常通过 |

```python
def should_stop(vehicle, tl, margin=3.0):
    if tl is None or tl.get_state() == carla.TrafficLightState.Green:
        return False
    dist = vehicle.get_location().distance(
        tl.get_stop_waypoints()[0].transform.location)
    return dist < margin or (
        tl.get_state() in [Red, Yellow] and dist < 15.0)
```

---

## 幻灯片 10: Traffic Manager 架构

```
╔═══════════════════════════════════════════╗
║          CARLA Traffic Manager            ║
║  ┌─────────────────────────────────────┐  ║
║  │  TM Agent (每辆车一个实例)          │  ║
║  │  ┌─────┐ ┌──────┐ ┌──────────┐    │  ║
║  │  │感知 │→│决策  │→│ 控制     │    │  ║
║  │  └─────┘ └──────┘ └──────────┘    │  ║
║  └─────────────────────────────────────┘  ║
║  ┌─────────────────────────────────────┐  ║
║  │  可配置参数                        │  ║
║  │  目标速度 │ 跟车距离 │ 闯灯概率   │  ║
║  │  变道行为 │ 限速忽略 │ 强行变道   │  ║
║  └─────────────────────────────────────┘  ║
╚═══════════════════════════════════════════╝
```

**用途**: 控制背景交通流，营造真实路况。
**特点**: 内置驾驶策略，无需自研算法。

---

## 幻灯片 11: Traffic Manager 参数配置

```python
# 激进驾驶员
tm.set_desired_speed(vehicle, 80.0)
tm.set_distance_to_leading_vehicle(vehicle, 2.0)
tm.set_ignore_traffic_light_percentage(vehicle, 20.0)
tm.set_lane_change_behavior(vehicle, 2)

# 保守驾驶员
tm.set_desired_speed(vehicle, 50.0)
tm.set_distance_to_leading_vehicle(vehicle, 8.0)
tm.set_ignore_traffic_light_percentage(vehicle, 0.0)
tm.set_lane_change_behavior(vehicle, 1)
```

| 参数 | 激进 | 普通 | 保守 |
|------|:----:|:----:|:----:|
| 目标速度 (km/h) | 80 | 60 | 40 |
| 跟车距离 (m) | 2.0 | 5.0 | 8.0 |
| 闯灯概率 (%) | 20 | 5 | 0 |
| 忽略限速 (%) | 30 | 5 | 0 |
| 变道行为 | 频繁 | 正常 | 保守 |

---

## 幻灯片 12: Scenario Runner 架构

```
┌──────────────────────────────────────────────────┐
│                  Scenario Runner                  │
├──────────────────────────────────────────────────┤
│ ┌──────────────┐  ┌──────────┐  ┌─────────────┐ │
│ │   Scenario   │  │  Agent   │  │  Evaluation  │ │
│ │  定义场景    │  │ 被测算法 │  │  评估指标    │ │
│ │  - 初始条件  │  │  FSM决策 │  │  碰撞检测    │ │
│ │  - 触发条件  │  │  MP轨迹  │  │  违规记录    │ │
│ │  - 终止条件  │  │  控制    │  │  完成任务    │ │
│ └──────────────┘  └──────────┘  └─────────────┘ │
│ ┌──────────────────────────────────────────────┐ │
│ │  场景描述: OpenScenario 1.0 / Python API    │ │
│ └──────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────┘
         │                              │
         v                              v
┌─────────────────┐          ┌─────────────────┐
│ CARLA Simulator │          │  被测系统        │
│ 环境/物理/渲染  │          │  ROS2 / Autoware│
└─────────────────┘          └─────────────────┘
```

---

## 幻灯片 13: 自定义场景示例

**场景: 行人横穿 (Pedestrian Crossing)**

```
    时间线:
    t=0s    自车以 10m/s 巡航
    t=3s    自车距斑马线 30m
    t=5s    行人出现在右侧人行道
    t=6s    行人开始横穿马路
    t=7s    自车检测到行人 ▸ 紧急制动
    t=9s    行人通过，自车恢复巡航
```

```python
class PedestrianCrossing(BasicScenario):
    def _create_behavior(self):
        root = py_trees.composites.Sequence("Behavior")
        approach = WaypointReached(
            self.ego_vehicles[0], self._trigger_point, 5.0)
        cross = CrossPedestrian(
            self._pedestrian, self._target_point)
        root.add_child(approach)
        root.add_child(cross)
        return root
```

---

## 幻灯片 14: 本章总结

```
┌──────────────────────────────────────────────────┐
│                  关键知识点                       │
├──────────────────────────────────────────────────┤
│  ✅ 行为决策是自动驾驶的"大脑"                    │
│  ✅ FSM 是实现决策的经典方法                     │
│  ✅ 交通灯响应需要停止线 + 灯色联合判断          │
│  ✅ Traffic Manager 控制背景交通流                │
│  ✅ Scenario Runner 测试决策算法                  │
├──────────────────────────────────────────────────┤
│                  实践任务                         │
├──────────────────────────────────────────────────┤
│  📝 练习43.1: 交通灯检测与停止                    │
│  📝 练习43.2: FSM 行为决策节点                   │
│  📝 练习43.3: Traffic Manager 交通流配置          │
└──────────────────────────────────────────────────┘
```
