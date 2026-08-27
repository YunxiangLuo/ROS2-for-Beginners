---
marp: true
theme: uncover
class:
  - lead
  - invert
---

# 第40章 车辆纵横向控制

## PID · Pure Pursuit · Stanley

---

# 幻灯片1：PID控制基本公式

$$ u(t) = K_p e(t) + K_i \int_0^t e(\tau)d\tau + K_d \frac{de(t)}{dt} $$

- **P** — 比例，对当前误差即时响应
- **I** — 积分，消除稳态误差
- **D** — 微分，预测误差趋势

```python
def pid(error, integral, derivative):
    return Kp*error + Ki*integral + Kd*derivative
```

---

# 幻灯片2：P/I/D项对系统影响

| 参数 | 上升时间 | 超调量 | 调节时间 | 稳态误差 |
|------|---------|--------|---------|---------|
| $K_p$ ↑ | 减小 | 增大 | 增大 | 减小 |
| $K_i$ ↑ | 减小 | 增大 | 增大 | 明显减小 |
| $K_d$ ↑ | 影响小 | 减小 | 减小 | 影响小 |

---

# 幻灯片3：Ziegler-Nichols 整定法

**临界比例法步骤**：
1. 仅用P控制，增大 $K_p$ 直至等幅振荡
2. 记录临界增益 $K_u$ 和周期 $T_u$
3. 查表：

| 类型 | $K_p$ | $K_i$ | $K_d$ |
|------|-------|-------|-------|
| P | $0.5K_u$ | — | — |
| PI | $0.45K_u$ | $0.54K_u/T_u$ | — |
| PID | $0.6K_u$ | $1.2K_u/T_u$ | $0.075K_uT_u$ |

---

# 幻灯片4：纵向控制总体架构

```
目标速度 → [速度PID] → 油门/刹车 → 车辆 → 实际速度
                  ↑                         |
                  └──── 速度反馈 ←──────────┘
```

**控制目标**：最小化速度跟踪误差

$$ e_v(t) = v_{target}(t) - v_{actual}(t) $$

---

# 幻灯片5：油门/刹车映射策略

```
PID输出:  u ∈ [-1, 1]
              │
       u ≥ 0  │  u < 0
         ↓    │    ↓
    throttle  │  brake
    = u       │  = -u
    brake=0   │  throttle=0
```

死区处理：$|e_v| < \epsilon \Rightarrow$ 输出置零

---

# 幻灯片6：Pure Pursuit 几何原理

```
                 目标路径
    预瞄点  ●——————————★———————
           ╱ ╲    θ
          ╱   ╲
         ╱  δ  ╲
        ╱       ╲   Ld
       ●—————————————
     车辆位置     预瞄距离
```

$$ \delta = \arctan\left(\frac{2L\sin\theta}{L_d}\right) $$

---

# 幻灯片7：Pure Pursuit 预瞄距离

**自适应预瞄距离**：
$$ L_d = L_{d0} + k_v \cdot v $$

- 低速 → 短预瞄 → 响应快，适合弯道
- 高速 → 长预瞄 → 平缓，适合直道

**参数敏感性**：
- 预瞄太短 → 振荡
- 预瞄太长 → 切弯

---

# 幻灯片8：Stanley 控制器

$$ \delta = \theta_e + \arctan\left(\frac{k \cdot e}{v + \epsilon}\right) $$

- $\theta_e$ — 航向误差
- $e$ — 横向偏移
- $k$ — 增益
- $v$ — 速度

**优势**：指数收敛到路径，弯道性能优于Pure Pursuit

---

# 幻灯片9：Pure Pursuit vs Stanley

| 对比项 | Pure Pursuit | Stanley |
|--------|-------------|---------|
| 原理 | 几何追踪 | 航向+横向反馈 |
| 收敛性 | 有稳态偏置（弯道） | 指数收敛 |
| 参数数量 | 1（预瞄距离） | 1（增益k） |
| 低速性能 | 好 | 对噪声敏感 |
| 高速性能 | 需自适应 | 自动补偿 |

---

# 幻灯片10：纵向PID实现

```python
class SpeedPIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return (self.Kp * error +
                self.Ki * self.integral +
                self.Kd * derivative)
```

---

# 幻灯片11：Pure Pursuit实现

```python
def compute_steering(vehicle_pos, vehicle_yaw,
                     waypoints, Ld):
    nearest = find_nearest_waypoint(vehicle_pos, waypoints)
    target = find_lookahead_point(waypoints, nearest, Ld)
    angle = atan2(target.y - vehicle_pos.y,
                  target.x - vehicle_pos.x)
    theta = angle - vehicle_yaw
    return atan2(2 * L * sin(theta), Ld)
```

---

# 幻灯片12：CARLA仿真环境设置

```python
import carla

client = carla.Client('localhost', 2000)
world = client.get_world()
bp = world.get_blueprint_library().find('vehicle.tesla.model3')
vehicle = world.spawn_actor(bp, spawn_transform)
```

---

# 幻灯片13：手动调参经验法则

1. $K_p$ 从小到大，直到开始振荡
2. $K_d$ 从0增大，抑制超调
3. $K_i$ 从较小值增大，消除稳态误差
4. 反复迭代

**口诀**：P给力，D阻尼，I消差

---

# 幻灯片14：增益调度自适应PID

```python
if speed < 3.0:       Kp, Ki, Kd = 1.5, 0.2, 0.4
elif speed < 8.0:     Kp, Ki, Kd = 1.0, 0.1, 0.3
elif speed < 15.0:    Kp, Ki, Kd = 0.7, 0.05, 0.2
else:                 Kp, Ki, Kd = 0.5, 0.03, 0.15
```

---

# 幻灯片15：本章小结

- ✅ PID控制三大分量物理意义
- ✅ 纵向速度PID + 油门刹车映射
- ✅ Pure Pursuit 几何追踪原理
- ✅ Stanley 航向+横向综合控制
- ✅ CARLA 仿真环境调参实践

> "控制不是让系统不犯错，而是让系统快速纠正错误"
