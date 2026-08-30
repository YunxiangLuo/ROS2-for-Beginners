# 第30章 实验手册：安全验证与系统集成

> **对应理论章节**：第44章《安全验证与系统集成》  
> **实验课时**：3 课时  
> **实验代码**：`src/lab_code/ch30_lab/`

## 实验环境

| 项目 | 规格 |
|---|---|
| 操作系统 | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Gazebo | Sim Harmonic |
| CARLA | 0.9.16（需要时启用） |
| Python | 3.12+ |

先完成课程环境配置并编译安全相关包：

```bash
bash setup_course.sh --with-carla
source ~/.config/ros2-course/env.bash
cd "$ROS2_COURSE_WS"
colcon build --symlink-install --packages-select \
  av_carla_interfaces av_safety_monitor
source install/setup.bash
```

## 实际运行证据

真实运行的安全监控、故障注入和状态话题检查：

![ch30 安全监控运行输出](images/runtime/ch30_safety.gif)

原始录制：[ch30_safety.cast](images/runtime/ch30_safety.cast)。

## 练习 30.1：验证安全监控节点

`av_safety_monitor` 提供 TTC 预警、碰撞事件处理、轨迹安全监控和故障注入节点。

启动安全监控：

```bash
ros2 run av_safety_monitor safety_monitor --ros-args \
  -p ttc_threshold_warning:=4.0 \
  -p ttc_threshold_critical:=2.5 \
  -p ttc_threshold_emergency:=1.5
```

主要接口：

| 方向 | 接口 |
|---|---|
| 订阅 | `/carla/ego_vehicle/collision` (`CollisionEvent`) |
| 订阅 | `/perception_objects` (`PerceptionObjectArray`) |
| 订阅 | `/plan` (`nav_msgs/Path`) |
| 订阅 | `/ego_state` (`geometry_msgs/TwistStamped`) |
| 发布 | `/safety_status` (`std_msgs/String`) |
| 发布 | `/safety_markers` (`visualization_msgs/MarkerArray`) |
| 发布 | `/emergency_stop` (`std_msgs/Bool`) |

检查节点和状态：

```bash
ros2 node list
ros2 topic list | grep -E 'safety|perception|plan|ego_state'
ros2 topic echo /safety_status
ros2 topic echo /emergency_stop
```

## 练习 30.2：故障注入

故障注入器当前针对 `std_msgs/String` 话题，适合先使用一个调试字符串话题验证丢帧、噪声、偏置、延迟和失效逻辑：

```bash
ros2 run av_safety_monitor fault_injector --ros-args \
  -p fault_type:=drop \
  -p target_topic:=/debug_status \
  -p fault_probability:=0.1
```

控制故障注入：

```bash
ros2 service call /inject_fault std_srvs/srv/SetBool "{data: false}"
ros2 topic echo /debug_status_injected
```

真实 `PerceptionObjectArray`、`Path` 等强类型话题不能直接套用该 String 注入器；需要为对应消息类型编写适配器后再做端到端故障测试。

## 练习 30.3：集成测试和性能评估

`src/labs/ch30_lab/` 提供独立的 ROS 2 测试和离线评估脚本：

```bash
cd "$ROS2_COURSE_WS/src/labs/ch30_lab"

# 运行单个场景
python3 integration_test.py --scenario straight_lane --duration 120

# 运行全部场景和故障配置
python3 integration_test.py --run-all

# 读取结果并生成 Markdown/JSON 报告
python3 eval_metrics.py --log-dir results/ch30_eval --format both
python3 eval_metrics.py --log-dir results/ch30_eval --summary
```

若使用 CARLA，先启动 `robot_sim_demo` 或 CARLA Bridge，再执行集成测试。测试脚本会记录速度、TTC、偏离、碰撞、舒适性和控制误差等指标。

## 单元测试

```bash
cd "$ROS2_COURSE_WS"
python3 -m pytest src/course/av_safety_monitor/test -q
```

## 验收清单

| 检查项 | 完成 |
|---|---|
| `av_safety_monitor` 可以正常启动 | □ |
| 碰撞事件会触发 `/emergency_stop` | □ |
| TTC 预警等级符合参数阈值 | □ |
| `/safety_status` 和 `/safety_markers` 正常发布 | □ |
| 故障注入器可启停并发布注入结果 | □ |
| 单元测试通过 | □ |
| 集成测试结果已生成 | □ |
| 报告记录了环境版本和未满足的依赖 | □ |

当前课程仓库不包含旧的独立自动驾驶包或统一集成 bringup 包；应以实际存在的 `av_*` 包和工作空间中的 `src/labs/ch30_lab/` 脚本为准。完整 CARLA 端到端验证需要 Ubuntu 24.04、ROS 2 Jazzy、CARLA 0.9.16 及对应运行依赖。
