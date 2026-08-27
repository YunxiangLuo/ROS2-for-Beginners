---
marp: true
theme: default
class: lead
paginate: true
---

# 第41章 多传感器融合定位

## LiDAR + IMU + GNSS 融合定位

**ROS 2 Jazzy + CARLA 0.9.16 仿真**

---

# Slide 2: 定位技术概述

## 为什么需要多传感器融合？

| 场景 | GNSS | IMU | LiDAR | 视觉 |
|------|------|-----|-------|------|
| 开阔道路 | ✅ | ❌(漂移) | ✅ | ✅ |
| 隧道/地库 | ❌ | ❌(漂移) | ✅ | ❌ |
| 雨雾天气 | ⚠️ | ✅ | ❌ | ❌ |
| 高速运动 | ✅ | ✅ | ⚠️ | ⚠️ |

**结论**: 单一传感器无法全场景覆盖, 必须融合

---

# Slide 3: 传感器特性对比

| 特性 | GNSS-RTK | IMU | LiDAR SLAM |
|------|---------|-----|------------|
| **精度** | 2~5 cm | 姿态0.1°, 位置发散 | 2~10 cm |
| **频率** | 10 Hz | 200 Hz | 10 Hz |
| **短期** | ❌ 多径 | ✅ 高频稳定 | ✅ 特征匹配 |
| **长期** | ✅ 无漂移 | ❌ 积分发散 | ⚠️ 累积漂移 |

**融合策略**: IMU高频驱动预测 + LiDAR/GNSS定期校正

---

# Slide 4: EKF 算法原理

## 扩展卡尔曼滤波流程

```
┌─────────────────────────────────────┐
│           预测步 (IMU驱动)            │
│  x̄ₖ = f(x̂ₖ₋₁, uₖ)                   │
│  P̄ₖ = F·Pₖ₋₁·Fᵀ + Q                │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│    更新步 (LiDAR / GNSS 观测)        │
│  Kₖ = P̄ₖ·Hᵀ(H·P̄ₖ·Hᵀ + R)⁻¹         │
│  x̂ₖ = x̄ₖ + Kₖ(zₖ - h(x̄ₖ))           │
│  Pₖ = (I - Kₖ·H) · P̄ₖ               │
└──────────────┬──────────────────────┘
               ▼
        ┌─────────────┐
        │ 输出 x̂ₖ, Pₖ │
        └─────────────┘
```

---

# Slide 5: 状态向量设计

## 15维 EKF 状态

| 分量 | 描述 | 单位 | 驱动源 |
|------|------|------|--------|
| x, y, z | 位置 | m | GNSS/LiDAR |
| roll, pitch, yaw | 姿态 | rad | LiDAR/IMU |
| vx, vy, vz | 线速度 | m/s | IMU |
| ωx, ωy, ωz | 角速度 | rad/s | IMU |
| ax, ay, az | 线加速度 | m/s² | IMU |

**观测更新矩阵**:
- LiDAR: 位置 + 姿态 (6-DOF)
- GNSS: 位置 x,y,z (3-DOF)

---

# Slide 6: 协方差矩阵设计

## 过程噪声 Q

```
Q = diag(σₚ²I₃, σᵧ²I₃, σᵥ²I₃, σω²I₃, σₐ²I₃)
  σₚ = 0.01    # 位置噪声
  σᵧ = 0.005   # 姿态噪声
  σᵥ = 0.1     # 速度噪声
  σω = 0.1     # 角速度噪声
  σₐ = 0.5     # 加速度噪声
```

## 观测噪声 R

```
R_lidar = diag(0.05², 0.05², 0.05², 0.01², 0.01², 0.02²)
R_gnss  = diag(0.10², 0.10², 0.15²)
```

---

# Slide 7: 松耦合融合架构

```
┌─────────┐  /carla/lidar   ┌─────────────┐
│ CARLA   │────────────────▶│  FAST-LIO2  │
│ 仿真器   │                 │  /odom_lidar│
│         │  /carla/imu     │             │
│         │────────────────▶│  EKF 15维   │
│         │                 │  robot_     │
│         │  /carla/gnss    │  localiz-   │
│         │────────────────▶│  ation      │
└─────────┘                 └──────┬──────┘
                                   │
                           /odometry/filtered
                                   │
                                   ▼
                          ┌────────────────┐
                          │ 定位评估节点    │
                          │ vs GT ATE/RPE  │
                          └────────────────┘
```

---

# Slide 8: TF2 坐标树

## 坐标系变换关系

```
map ──(初始化)──▶ odom ──(EKF)──▶ base_link
                                   │
                     ┌─────────────┼─────────────┐
                     ▼             ▼             ▼
                lidar_link      imu_link      gps_link
                (x:0, y:0,    (x:0.5,       (x:0.8,
                 z:1.8)        y:0, z:0.3)    y:0, z:0.1)
```

**IMU→base_link** 标定示例:
```yaml
# static_transform_publisher
x: 0.5, y: 0.0, z: 0.3
roll: 0.0, pitch: 0.0, yaw: 0.0
parent: base_link, child: imu_link
```

---

# Slide 9: robot_localization 配置

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: false
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # IMU 驱动预测
    imu0: /carla/imu
    imu0_config: [true, true, true,  # 加速度
                  true, true, true,  # 角速度
                  false, false, false,
                  false, false, true,
                  true, true, true]

    # LiDAR SLAM 位姿观测
    odom0: /odom_lidar
    odom0_config: [true, true, true,
                   true, true, true,
                   false, false, false,
                   false, false, false,
                   false, false, false]
```

---

# Slide 10: 频率对齐策略

## 多传感器异步融合

```
IMU ────┬───┬───┬───┬───┬───┬───┬───┬───┬─── (200 Hz)
        │   │   │   │   │   │   │   │   │
        ▼   ▼   ▼   ▼   ▼   ▼   ▼   ▼   ▼
      预测 预测 预测 更新 预测 预测 预测 预测 更新 (30 Hz)

LiDAR ──────────┬────────────────────┬────── (10 Hz)
                │ 位姿观测            │
GNSS  ─────────────┬──────────────────── (5 Hz)
                    │ 位置观测
```

**策略**:
- 每个 IMU 消息触发 EKF 预测步
- LiDAR SLAM 位姿到达时触发 6-DOF 更新
- GNSS 到达时触发 3-DOF 位置更新

---

# Slide 11: 异常观测检测

## 马氏距离异常值滤除

```python
χ²_95 = {3: 7.815, 6: 12.592}  # 95%置信度阈值

innovation = z - h(x̄)
S = H @ P̄ @ H.T + R
d² = innovation.T @ inv(S) @ innovation

if d² > χ²_95[dim]:
    reject()  # LiDAR退化或GNSS多径
else:
    update()   # 正常观测
```

---

# Slide 12: CARLA Ground Truth

## 获取真值数据

```python
# CARLA 提供精确真值
class CarlaGtNode(Node):
    def timer_callback(self):
        vehicle = self.world.get_actors().filter('vehicle.*')[0]
        t = vehicle.get_transform()
        v = vehicle.get_velocity()
        
        odom = Odometry()
        odom.pose.pose.position.x = t.location.x
        odom.pose.pose.orientation.w = t.rotation.yaw
        # ... 发布 /carla/ground_truth
```

**注意**: CARLA 真值基于仿真引擎内部状态, 精度可达微米级, 可作为定位评估的标准参考。

---

# Slide 13: 评估指标 ATE / RPE

| 指标 | 公式 | 意义 |
|------|------|------|
| **ATE RMSE** | $\sqrt{\frac{1}{N}\sum\|p_{est} - p_{gt}\|^2}$ | 绝对轨迹误差 |
| **RPE trans** | $\frac{1}{M}\sum\|trans(e_{ij})\|$ | 平移漂移率 |
| **RPE rot** | $\frac{1}{M}\sum\|rot(e_{ij})\|$ | 旋转漂移率 |

**使用 evo 工具评估**:
```bash
evo_ape tum gt.tum est.tum -a --plot --plot_mode xy
evo_rpe tum gt.tum est.tum -a --delta 1.0 --plot
```

**精度目标**: ATE RMSE < 0.20m (城区), RPE < 0.03 m/m

---

# Slide 14: 本章小结

## 关键知识点

1. ✅ **EKF 融合框架**: 预测-更新结构, 15维状态向量
2. ✅ **松耦合架构**: LiDAR/IMU/GNSS 各自独立处理
3. ✅ **TF2 坐标树**: map→odom→base_link→sensor
4. ✅ **robot_localization**: 成熟的ROS2 EKF实现
5. ✅ **CARLA 评估**: ATE/RPE 指标, evo 工具

## 下一步

- **Lab 41.1**: 构建TF2坐标树
- **Lab 41.2**: 实现EKF融合定位
- **Lab 41.3**: 精度评估与可视化

---

> **参考资源**: robot_localization Wiki, FAST-LIO2 GitHub, EVO 文档, CARLA 官方教程
