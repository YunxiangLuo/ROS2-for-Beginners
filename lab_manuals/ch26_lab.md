# 第26章 实验指导：车辆纵横向控制

> **对应理论章节**：第40章《车辆纵横向控制（PID）》  
> **实验课时**：3 课时  
> **实验代码**：`src/lab_code/ch26_lab/`

## 实验目标

1. 实现基于PID的纵向速度控制器
2. 实现基于Pure Pursuit的横向控制器
3. 在CARLA仿真环境中完成路径跟踪闭环测试

---

## 练习26.1：实现纵向PID控制器

### 任务描述

实现一个完整的纵向速度PID控制器，包含抗积分饱和（Anti-windup）和微分滤波功能。

### 要求

- 实现 `pid_controller.py` 中的 `PIDController` 类
- 支持积分限幅和输出限幅
- 微分项带一阶低通滤波
- 支持速度→油门/刹车的映射

### 代码框架

```python
class PIDController:
    def __init__(self, Kp, Ki, Kd, dt=0.05):
        # TODO: 初始化参数和状态变量
        pass

    def reset(self):
        # TODO: 重置积分和微分状态
        pass

    def compute(self, target, current):
        # TODO: 计算PID输出
        # 1. 计算误差
        # 2. 更新积分项（带限幅）
        # 3. 计算微分项（带滤波）
        # 4. 组合输出并限幅
        pass
```

### 测试方法

```python
# 在终端运行单元测试
python pid_controller.py --test
```

### 验收标准

- 阶跃响应无稳态误差（$|e| < 0.1 m/s$）
- 超调量 $< 10\%$
- 调节时间 $< 3s$

---

## 练习26.2：实现Pure Pursuit横向控制器

### 任务描述

实现Pure Pursuit路径跟踪算法，包含预瞄距离自适应和转向角计算。

### 要求

- 实现 `pure_pursuit.py` 中的 `PurePursuit` 类
- 预瞄距离随速度自适应
- 查找最近路径点和预瞄点
- 计算期望前轮转角

### 代码框架

```python
class PurePursuit:
    def __init__(self, wheelbase, Ld0=2.0, kv=0.5):
        # TODO: 初始化车辆参数
        pass

    def find_lookahead_point(self, waypoints, vehicle_pos):
        # TODO: 找到预瞄距离内的目标点
        pass

    def compute_steering(self, vehicle_pos,
                         vehicle_yaw, waypoints):
        # TODO: 计算前轮转角
        pass
```

### 测试方法

```python
python pure_pursuit.py --test
```

### 验收标准

- 直线跟踪横向偏差 $< 0.2m$
- 弯道跟踪横向偏差 $< 0.5m$
- 不同速度下控制稳定

---

## 练习26.3：在CARLA中完成路径跟踪测试

### 任务描述

将纵向和横向控制器集成到 `vehicle_ctrl.py`，在CARLA仿真环境中完成路径跟踪。

### 要求

1. 启动CARLA服务器并加载地图 `Town03`
2. 使用 `lat_lon_ctrl.py` 作为主控节点
3. 设定10个路径点的环形路径
4. 控制车辆以 36km/h (10m/s) 目标速度循迹
5. 记录并输出跟踪误差数据

### 运行步骤

```bash
# Step 1: 启动CARLA服务器
# Step 2: 运行控制节点
python lat_lon_ctrl.py
```

### 预期输出

```
[INFO] 启动车辆纵横向控制器
[INFO] 目标速度: 10.0 m/s
[INFO] 目标路径点: 10个
[INFO] --- 第1帧 ---
[INFO] 实际速度: 1.23 m/s, 速度误差: 8.77 m/s
[INFO] 横向偏差: 1.45 m, 航向误差: 12.34 deg
[INFO] 油门: 0.85, 刹车: 0.00, 方向盘: -0.12
...
[INFO] === 测试完成 ===
[INFO] 平均速度误差: 0.45 m/s
[INFO] 平均横向偏差: 0.28 m
[INFO] 最大横向偏差: 0.62 m
```

### 扩展挑战（可选）

1. 增加Stanley控制器，与Pure Pursuit对比
2. 实现增益调度PID，在不同速度下切换参数
3. 增加前馈控制项，改善弯道性能
4. 使用ROS2话题发布控制状态，用rqt可视化

---

## 提交要求

1. 完整实现 `pid_controller.py`, `pure_pursuit.py`, `vehicle_ctrl.py`, `lat_lon_ctrl.py`
2. 提供CARLA测试录屏或日志文件
3. 提交实验报告（PDF），包含：
   - 控制器参数及调试过程
   - 关键代码段说明
   - 测试结果截图和分析

## 评分标准

| 项目 | 分值 | 要求 |
|------|------|------|
| 纵向PID实现 | 30分 | 完整实现、抗积分饱和、无稳态误差 |
| Pure Pursuit实现 | 30分 | 预瞄自适应、弯道跟踪稳定 |
| CARLA集成测试 | 30分 | 闭环跟踪、误差在范围内 |
| 实验报告 | 10分 | 分析充分、数据完整 |

## 实际运行证据

当前环境已实际运行 PID 和 Pure Pursuit 控制器测试；CARLA 服务端和 Bridge 已安装，但 UE4 在当前 WSL 图形后端初始化时退出，因此 CARLA 闭环部分暂未登记截图。

![ch26 控制器测试输出](images/runtime/ch26_control.gif)

原始录制：[ch26_control.cast](images/runtime/ch26_control.cast)。完整证据索引见[实际运行证据](runtime_evidence.md)。
