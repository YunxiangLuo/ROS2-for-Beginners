# 第10章 SLAM 建图与定位

## 第1页：章节目录
- 10.1 SLAM 基本概念
- 10.2 GraphSLAM 图优化
- 10.3 粒子滤波 SLAM
- 10.4 slam_toolbox 在线/离线建图
- 10.5 AMCL 蒙特卡洛定位
- 10.6 map_server 地图管理
- 10.7 Cartographer 多传感器 SLAM
- 10.8 仿真建图演示
- 10.9 练习与总结

---

## 第2页：什么是 SLAM
**SLAM = Simultaneous Localization and Mapping**

```
         ┌──────────┐    观测 z    ┌──────────┐
         │ 真实环境 │────────────>│  传感器  │
         └──────────┘             └────┬─────┘
                                      │
         ┌─────────────────────────────▼──┐
         │        SLAM 系统              │
         │   ┌───────┐    ┌────────┐     │
         │   │定位模块│<──>│建图模块│     │
         │   └───┬───┘    └───┬────┘     │
         │       │            │          │
         │   ┌───▼────────────▼───┐     │
         │   │   数据关联/回环    │     │
         │   └────────────────────┘     │
         └──────────────────────────────┘
                    │
              ┌─────▼─────┐
              │ 输出: 地图+位姿 │
              └─────────────┘
```

**核心挑战:** 鸡生蛋蛋生鸡 — 定位需要地图, 建图需要位姿

---

## 第3页：GraphSLAM 图优化 (1/2)
**图优化 = 构建图 + 求解**

```
节点 (Vertex): 机器人位姿 x_t, 路标点 l_j
边 (Edge):  约束 (观测/里程计) 连接节点

位姿图示意:
  x0 ──odom──> x1 ──odom──> x2 ──odom──> x3
  │            │             │            │
  obs          obs          obs          obs
  │            │             │            │
  l1           l2            l3           l4

回环边:
  x3 ----loop_closure----> x0  (闭合约束)
```

**优化目标:** min Σ ||f(x_i, x_j) - z_ij||²_Ω
- f 为约束函数, z_ij 为测量值, Ω 为信息矩阵

---

## 第4页：GraphSLAM 求解方法 (2/2)
**主流求解器方案:**

| 方法 | 特点 | ROS 2 对应 |
|------|------|------------|
| 高斯-牛顿 | 一阶近似, 速度快 | slam_toolbox |
| Levenberg-Marquardt | 信赖域控制, 鲁棒 | Cartographer 后端 |
| Dogleg | 梯度下降+牛顿混合 | Ceres 默认 |
| iSAM2 | 增量平滑 | GTSAM / LIO-SAM |

```
SLAM 前端 → 特征提取、scan matching、数据关联
SLAM 后端 → 图构建、非线性优化、回环检测
```

---

## 第5页：粒子滤波 SLAM (FastSLAM)
**Rao-Blackwellized 粒子滤波:**

```
粒子 1: [轨迹] + [路标 EKF: l1, l2, l3...]
粒子 2: [轨迹] + [路标 EKF: l1, l2, l3...]
粒子 3: [轨迹] + [路标 EKF: l1, l2, l3...]
...
粒子 N: [轨迹] + [路标 EKF: l1, l2, l3...]

分解: p(x_1:t, m | z_1:t, u_1:t-1)
     = p(x_1:t | z, u) × Π p(l_j | x_1:t, z, u)
         ^粒子滤波            ^每个粒子独立 EKF
```

**流程:**
1. 采样：从提议分布采样新位姿
2. 更新：EKF 更新路标点估计
3. 加权：计算粒子重要性权重
4. 重采样：根据权重复制/淘汰粒子

---

## 第6页：slam_toolbox 在线/离线建图
**两种建图模式:**

```
在线异步建图 (Online Async):
  ┌──────────┐   scan   ┌───────────────┐
  │ 激光雷达 │─────────>│ slam_toolbox  │──> /map
  └──────────┘          │               │──> /map_updates
  ┌──────────┐   odom   │ 在线 SLAM     │──> tf map→odom
  │ 里程计   │─────────>│               │
  └──────────┘          └───────────────┘

离线建图 (Offline / Record Bag):
  ros2 bag record → 事后 playback → 高质量地图
```

**启动命令:**
```bash
# 在线
ros2 launch slam_toolbox online_async_launch.py
# 离线
ros2 launch slam_toolbox offline_launch.py bag_file:=my_bag
```

---

## 第7页：AMCL 蒙特卡洛定位
**自适应蒙特卡洛定位:**

```
AMCL 工作流程:
  1. 初始化粒子集 (均匀分布或正态分布)
  2. 运动更新 (里程计模型 + 高斯噪声)
  3. 测量更新 (激光似然场模型, 更新权重)
  4. KLD 重采样 (自适应粒子数)
  5. 发布估计位姿 → /amcl_pose
```

**KLD 采样:**
```
当 N > (k-1)/(2ε) * (1 - 2/(9(k-1)) + sqrt(2/(9(k-1)))*z)^3
自适应暂停重采样: 粒子已足够集中
```

**关键话题:**
- `/initialpose` — 设置初始位姿 (rviz 2D Pose Estimate)
- `/amcl_pose` — 估计位姿 + 协方差
- `/particlecloud` — 可视化粒子集合

---

## 第8页：map_server 地图管理
**地图文件格式:**

```yaml
# my_map.yaml
image: my_map.pgm
resolution: 0.05       # m/pixel
origin: [-10.0, -10.0, 0.0]
negate: 0              # 0: 白=空闲
occupied_thresh: 0.65
free_thresh: 0.196
mode: trinary          # trinary | scale | raw
```

**占用栅格地图 (OccupancyGrid):**
```
值含义:  0   = 空闲 (白色)
        100 = 占据 (黑色)
        -1  = 未知 (灰色)

  □ □ □ ■ ■
  □ □ □ ■ ■
  □ ■ ■ ■ ■
  □ □ □ □ ■
```

**命令行操作:**
```bash
ros2 run nav2_map_server map_saver_cli -f my_map   # 保存
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=my_map.yaml  # 加载
```

---

## 第9页：Cartographer 多传感器 SLAM
**Google Cartographer 架构:**

```
 激光  ──┐
 IMU   ──┼──> 局部SLAM ──> Submap ──┐
 里程计──┘  (scan matching)         │
                                     ├──> 全局SLAM (回环+SPA优化)
                                     │
                                       ──> 最终地图

子图 (Submap): 短时间内的局部地图
  - 每个子图 ~100 帧扫描
  - 使用栅格概率地图表示
  - 通过 Ceres 求解器做 scan-to-map 匹配
```

**多楼层建图策略:**
- 使用 multi-floor 轨迹
- 楼层间切换时重置子图
- 使用 barometer 或 elevator 检测作楼层变换信号

---

## 第10页：Gazebo 仿真建图演示
**slam_sim_demo_ros2 包结构:**

```
slam_sim_demo_ros2/
├── launch/
│   ├── gazebo_world.launch.py    # 启动 Gazebo 世界
│   ├── slam_toolbox.launch.py    # 启动 SLAM
│   └── rviz_view.launch.py       # 启动可视化
├── worlds/
│   └── office.world              # 办公场景
├── maps/
│   └── office_map.yaml           # 参考地图
└── config/
    └── mapper_params.yaml        # SLAM 参数
```

**建图流程:**
```bash
# 终端1: 启动仿真
ros2 launch slam_sim_demo_ros2 gazebo.launch.py
# 终端2: 启动 SLAM
ros2 launch slam_sim_demo_ros2 slam.launch.py
# 终端3: teleop 控制机器人探索
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 终端4: 完成后保存地图
ros2 run nav2_map_server map_saver_cli -f ~/maps/office_map
```

---

## 第11页：SLAM 方案对比
| 特性 | slam_toolbox | Cartographer | ORB-SLAM3 |
|------|-------------|-------------|-----------|
| 传感器 | 2D 激光 | 2D/3D 激光+IMU | 单目/双目/RGB-D |
| 优化 | Karto 图优化 | SPA 图优化 | 位姿图+BA |
| 回环 | Ceres sparse | 分支定界+CSM | DBoW2 词袋 |
| 地图 | 栅格地图 | 栅格+子图 | 稀疏点云 |
| 实时性 | ★★★★☆ | ★★★★★ | ★★★★☆ |
| 易用性 | ★★★★★ | ★★★☆☆ | ★★★☆☆ |

---

## 第12页：本章总结
**核心要点回顾:**
1. SLAM = 同时定位与建图, 核心是概率推断
2. GraphSLAM 用图优化求解, 回环闭合是质量关键
3. slam_toolbox 适用于 2D 激光 SLAM, 易配置
4. AMCL 用粒子滤波实现已知地图下的全局定位
5. Cartographer 支持多传感器, 适合复杂环境

**下一步: 第11章 Nav2 自主导航**
