# 第29章 实验代码：行为决策与交通规则

## 代码结构

```
src/lab_code/ch29_lab/
├── README.md                       # 本文件
├── fsm_decision.py                 # 有限状态机行为决策
├── traffic_light_detector.py       # 交通灯检测与响应
└── traffic_manager_demo.py         # CARLA Traffic Manager 配置
```

## 文件说明

### fsm_decision.py

基于有限状态机的行为决策节点，包含三种状态：

- **CRUISE** - 车道保持巡航
- **FOLLOW** - 跟车模式
- **STOP** - 停车模式

状态转移逻辑：检测到前车时从 CRUISE → FOLLOW，前车消失时返回 CRUISE；检测到红灯时进入 STOP。

**运行：**

```bash

python fsm_decision.py
```

### traffic_light_detector.py

交通灯检测与响应节点，实现：

- 订阅 CARLA 交通灯状态主题
- 计算车辆到停止线的距离
- 根据灯色和距离输出油门/刹车控制

**运行：**

```bash
python traffic_light_detector.py
```

### traffic_manager_demo.py

CARLA Traffic Manager 配置演示，支持三种驾驶风格：

| 模式 | 特点 |
|------|------|
| normal | 普通驾驶，遵守规则 |
| aggressive | 激进驾驶，闯灯概率20% |
| conservative | 保守驾驶，跟车距离大 |

**运行：**

```bash

python traffic_manager_demo.py --vehicles 20 --mode normal

python traffic_manager_demo.py --vehicles 30 --mode aggressive

python traffic_manager_demo.py --vehicles 50 --mode mixed
```

## 依赖

- Python 3.8+
- carla (pip: `pip install carla`)
- ROS 2 Jazzy（若使用 ROS 2 版本）
- rclpy, carla_msgs (ROS2 包)

## 对应实验

- 练习 29.1: `traffic_light_detector.py`
- 练习 29.2: `fsm_decision.py`
- 练习 29.3: `traffic_manager_demo.py`

详细实验步骤见 `lab_manuals/ch29_lab.md`。

---

## 安装与编译

```bash
pip install numpy
# 需本机 CARLA 0.9.16 运行中
```

## 运行方法

```bash

python traffic_light_detector.py    # 交通灯检测与停车响应

python fsm_decision.py              # FSM 行为决策(CRUISE/FOLLOW/STOP/AVOID/COMPLETE)

python traffic_manager_demo.py      # 交通管理演示
```

## 验证

在 CARLA 运行时分别执行交通灯检测、FSM 和 Traffic Manager 脚本，观察巡航、跟车、停车和交通流行为。本目录未提供独立离线测试套件。
