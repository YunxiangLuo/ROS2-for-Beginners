# 第30章 实验代码：安全验证与系统集成

## 文件结构

```
src/lab_code/ch30_lab/
├── README.md              # 本文件
├── safety_monitor.py      # 安全监控节点：碰撞检测 + 偏离检测 + AEB
├── fault_injector.py      # 故障注入器：丢帧/噪声/偏置/延迟/失效
├── integration_test.py    # 集成测试框架：端到端自动化测试
└── eval_metrics.py        # 性能评估工具：指标计算与报告生成
```

## 依赖

```bash

pip install numpy matplotlib pandas scipy
```

ROS2包依赖：
- `rclpy`
- `std_msgs`, `geometry_msgs`, `nav_msgs`
- `visualization_msgs`
- `av_carla_interfaces`（提供 `PerceptionObjectArray` 等课程接口）

## 快速开始

```bash
# 1. 启动CARLA仿真
"$CARLA_ROOT/CarlaUE4.sh" -quality-level=Low

# 2. 启动ROS2桥接
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py \
  synchronous_mode:=False register_all_sensors:=True

# 3. 启动安全监控（课程 ROS 2 包）
ros2 run av_safety_monitor safety_monitor

# 4. 启动故障注入（当前实现针对 std_msgs/String）
ros2 run av_safety_monitor fault_injector --ros-args \
  -p fault_type:=drop -p target_topic:=/debug_status -p fault_probability:=0.05

# 5. 运行集成测试
python3 integration_test.py --scenario straight_lane --duration 120

# 6. 评估结果
python3 eval_metrics.py --log-dir results/ch30_eval --output report.md
```

## 话题接口

| 话题 | 类型 | 说明 |
|------|------|------|
| `/perception_objects` | `PerceptionObjectArray` | 感知目标列表 |
| `/plan` | `nav_msgs/Path` | 规划路径 |
| `/ego_state` | `geometry_msgs/TwistStamped` | 自车速度 |
| `/carla/ego_vehicle/collision` | `CollisionEvent` | 碰撞事件 |
| `/safety_status` | `std_msgs/String` | 安全状态 |
| `/safety_markers` | `MarkerArray` | 安全可视化 |
| `/emergency_stop` | `Bool` | 紧急停车 |
| `/debug_status_injected` | `String` | 注入后的调试话题 |

---

## 安装与编译

```bash

pip install numpy matplotlib
```

## 运行方法

```bash
python eval_metrics.py --log-dir results/ch30_eval --summary   # 汇总评估
python eval_metrics.py --log-dir results/ch30_eval             # 生成 md/json 报告
ros2 run av_safety_monitor safety_monitor
ros2 run av_safety_monitor fault_injector
python3 integration_test.py --scenario straight_lane --duration 120
```

## 验证

在 CARLA 与 ROS 2 Jazzy 环境中，运行 `integration_test.py` 生成日志后，可用 `eval_metrics.py` 汇总安全、舒适性、效率和实时性指标。本目录未提供独立离线测试套件。
