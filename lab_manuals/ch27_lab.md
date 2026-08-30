# 第27章 实验手册: 多传感器融合定位

> **对应理论章节**：第41章《多传感器融合定位》  
> **实验课时**：3 课时  
> **实验代码**：`src/lab_code/ch27_lab/`

**实验目标**: 掌握基于 robot_localization 的 LiDAR-IMU-GNSS 融合定位, 并利用 CARLA 真值评估精度。

---

## 实验环境

| 项目 | 版本/配置 |
|------|----------|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| CARLA | 0.9.16 |
| robot_localization | Jazzy 二进制安装 |
| EVO | pip install evo |
| Python | 3.10+ |

```bash
# 安装依赖
sudo apt install ros-jazzy-robot-localization
pip install evo --upgrade --no-deps
pip install numpy matplotlib pandas
```

---

## 练习27.1: 构建 TF2 坐标树

### 目标

编写 TF2 广播节点, 发布 `map → odom → base_link → lidar/imu/gps` 的坐标系变换。

### 步骤

#### Step 1: 创建 TF 广播节点

创建 `src/lab_code/ch27_lab/tf_broadcaster.py` (已提供模板):

```python
# 主要职责:
# 1. 发布 base_link → lidar_link (static)
# 2. 发布 base_link → imu_link (static)
# 3. 发布 base_link → gps_link (static)
# 4. 接收 /ekf_odom 并转发 map→odom→base_link
```

#### Step 2: 启动 TF 树

```bash
# 启动 TF 树 (终端1)
python3 tf_broadcaster.py

# 查看 TF 树 (终端2)
ros2 run tf2_tools view_frames.py
```

#### Step 3: 验证坐标变换

```bash
# 查看所有变换
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_ros tf2_echo base_link imu_link

# 预期输出: 各变换的平移/旋转分量与配置一致
```

### 实验检查点

| 检查项 | 预期结果 |
|--------|---------|
| `tf2_echo map base_link` | 显示EKF估计的相对位姿 |
| `tf2_echo base_link lidar_link` | 显示固定变换 (x:0, y:0, z:1.8) |
| `tf2_echo base_link imu_link` | 显示固定变换 (x:0.5, y:0, z:0.3) |
| `tf2_echo base_link gps_link` | 显示固定变换 (x:0.8, y:0, z:0.1) |
| `view_frames` PDF | 包含所有5个坐标系节点 |

---

## 练习27.2: 实现 EKF 融合节点

### 目标

使用 robot_localization 配置 EKF, 融合 IMU 预测 + LiDAR SLAM 位姿 + GNSS 位置。

### 步骤

#### Step 1: 配置 EKF 参数

编辑 `src/lab_code/ch27_lab/ekf_localization.yaml`, 重点关注:

1. **频率设置**: `frequency: 30.0`
2. **坐标系**: `world_frame: odom`, `odom_frame: odom`, `map_frame: map`
3. **IMU 配置**: 启用加速度 + 角速度 + 方向
4. **LiDAR 配置**: 启用位置 + 姿态 (6-DOF)
5. **GNSS 配置**: 启用位置 x/y/z (3-DOF)

#### Step 2: 启动融合系统

```bash
# 先执行 `source ~/.config/ros2-course/env.bash`，并在另一终端运行 `carla-server`
# 终端1: CARLA + ROS2 桥接
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py

# 终端2: LiDAR SLAM (FAST-LIO2 或类似)
ros2 launch fast_lio mapping.launch.py

# 终端3: EKF 融合
ros2 launch robot_localization ekf.launch.py \
    params_file:=src/lab_code/ch27_lab/ekf_localization.yaml

# 终端4: TF 广播
python3 src/lab_code/ch27_lab/tf_broadcaster.py
```

#### Step 3: 监测输出

```bash
# 查看融合后的里程计
ros2 topic echo /odometry/filtered --once

# 检查 EKF 诊断信息
ros2 topic echo /diagnostics --filter "m.name == 'ekf_filter_node'"
```

### EKF 参数调优

| 参数 | 默认值 | 调优方向 | 效果 |
|------|-------|---------|------|
| `process_noise_covariance` | 默认 | 增大 | 更信任观测, 响应更快 |
| `odom0_queue_size` | 5 | 增大 | 减少丢帧 |
| `imu0_remove_gravitational_acceleration` | true | 保持 | 消除重力影响 |
| `print_diagnostics` | true | 保持调试 | 查看协方差收敛 |

### 常见问题

**Q: EKF 输出 NAN?**
A: 检查 IMU 数据是否包含加速度/角速度, 确认坐标系框架一致。

**Q: 位姿跳变?**
A: 查看 innovation 是否在阈值内, 可能需要增大 R 或减小 Q。

---

## 练习27.3: 对比 CARLA Ground Truth

### 目标

将 EKF 输出与 CARLA 提供的真值对比, 计算 ATE 和 RPE 指标。

### 步骤

#### Step 1: 录制数据

```bash
# 启动评估节点
python3 src/lab_code/ch27_lab/localization_eval.py

# 录制 bag (至少 60 秒)
ros2 bag record -o ch27_eval \
    /carla/ground_truth \
    /odometry/filtered
```

#### Step 2: 提取轨迹

```bash
# 提取为 TUM 格式
evo_traj bag ch27_eval/ch27_eval.db3 \
    /carla/ground_truth --save_as_tum gt.tum
evo_traj bag ch27_eval/ch27_eval.db3 \
    /odometry/filtered --save_as_tum est.tum
```

#### Step 3: ATE 评估

```bash
# 计算 ATE
evo_ape tum gt.tum est.tum -a -v --plot --plot_mode xy

# 输出示例:
# ATE RMSE: 0.186543 m
# ATE mean: 0.152341 m
# ATE std:  0.108724 m
# ATE min:  0.023456 m
# ATE max:  0.412358 m
```

#### Step 4: RPE 评估

```bash
# 计算 RPE (每1米)
evo_rpe tum gt.tum est.tum -a --delta 1.0 \
    --delta_unit m -v --plot --plot_mode xy
```

#### Step 5: 可视化分析

```python
# 使用评估脚本生成报告
python3 localization_eval.py --bag ch27_eval/ch27_eval.db3

# 输出文件:
#   trajectory_comparison.png  - 轨迹对比图
#   ate_error_map.png          - ATE 误差分布
#   evaluation_report.txt      - 详细评估报告
```

### 评估结果分析

```bash
# 查看报告
cat evaluation_report.txt

# 报告示例:
# ════════════════════════════════════════
#   定位精度评估报告
# ════════════════════════════════════════
#   数据集: ch27_eval/ch27_eval.db3
#   时长: 120.5 s
#   里程: 1863.2 m
#   ATE RMSE: 0.195 m  ← 评估核心指标
#   ATE mean: 0.162 m
#   ATE std:  0.109 m
#   RPE trans: 0.028 m/m  ← 漂移率
#   RPE rot:   0.015 deg/m
#   最大误差点: (x=123.4, y=567.8) @ t=45.2s
# ════════════════════════════════════════
```

### 精度对比基准

| 定位方案 | ATE RMSE | RPE trans | 适用场景 |
|---------|---------|-----------|---------|
| 纯 GNSS (单点) | 2~5 m | 0.2 m/m | 开阔区域 |
| GNSS-RTK | 0.05~0.15 m | 0.01 m/m | RTK覆盖区 |
| LiDAR SLAM | 0.10~0.30 m | 0.02 m/m | 特征丰富 |
| **EKF 融合 (本章)** | **0.10~0.30 m** | **<0.03 m/m** | **全场景** |

---

## 实验报告要求

### 提交内容

1. `tf_tree.pdf` - 练习27.1的 TF 树截图
2. `ekf_params.yaml` - 练习27.2的最终参数配置
3. `evaluation_report.txt` - 练习27.3的完整评估报告
4. `trajectory_comparison.png` - 轨迹对比图
5. 实验总结 (200字以上), 包含:

### 评分标准

| 项目 | 分数 | 标准 |
|------|------|------|
| TF树完整性 | 20分 | 包含所有5个坐标系 |
| EKF配置正确 | 30分 | IMU/LiDAR/GNSS均正确接入 |
| ATE评估 | 20分 | 正确计算并绘制ATE |
| 精度达标 | 20分 | ATE RMSE < 0.30m |
| 报告质量 | 10分 | 分析充分, 图表完整 |

### 思考题

1. 为什么 EKF 使用 `world_frame=odom` 而不是 `world_frame=map`?
2. 当 LiDAR SLAM 输出频率从 10Hz 降低到 1Hz 时, EKF 精度如何变化?
3. 如何检测并处理 GNSS 多径效应造成的异常跳变?
4. 紧耦合相比松耦合在精度上有什么优势? 为什么本章选择松耦合?

---

## 参考命令速查

```bash
# TF CLI
ros2 run tf2_tools view_frames.py
ros2 run tf2_ros tf2_echo <parent> <child>

# 话题列表
ros2 topic list
ros2 topic hz /odometry/filtered

# 录包
ros2 bag record -a -o my_bag

# 评估 (evo)
evo_ape tum gt.tum est.tum -a --plot
evo_rpe tum gt.tum est.tum -a --delta 1 -v

# 轨迹可视化 (多个轨迹对齐)
evo_traj tum gt.tum est.tum --ref gt.tum -a --plot
```

---

> **提示**: 如果使用 CARLA 仿真, 请确保在启动桥接前已加载地图并生成车辆。参考 `carla_ros_bridge` 文档获取详细启动步骤。
