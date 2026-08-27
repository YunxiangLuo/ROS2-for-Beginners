# 第12章 MoveIt 2 机械臂编程

## 第1页: 章节目录
- 12.1 MoveIt 2 架构概览
- 12.2 MoveItPy Python API
- 12.3 运动规划 (Kinematics + Planning)
- 12.4 正/逆运动学 (FK/IK)
- 12.5 规划场景与碰撞检测
- 12.6 抓取规划
- 12.7 ros2_control 集成
- 12.8 练习与总结

---

## 第2页: MoveIt 2 系统架构

```
┌────────────┐
│ RViz 插件   │──── 可视化交互 ────┐
└────────────┘                    │
┌────────────┐                    │
│ MoveItPy   │──── Python API ────┤
└────────────┘                    │
           ┌──────────────────────▼──────┐
           │     MoveGroupInterface      │
           │   - 规划/执行/状态查询       │
           └──────────────┬──────────────┘
        ┌─────────────────┼─────────────────┐
  ┌─────▼─────┐    ┌──────▼──────┐    ┌─────▼─────┐
  │ 运动学求解 │    │  碰撞检测    │    │  规划器    │
  │ KDL/TRAC  │    │  FCL/Bullet │    │ OMPL/CHOMP │
  └───────────┘    └─────────────┘    └───────────┘
```

**核心组件:**
- MoveGroup: 统一接口 (类似 Nav2 BT)
- Planning Scene Monitor: 实时维护环境模型
- ros2_control: 硬件抽象层

---

## 第3页: MoveItPy 快速入门

```
基本调用流程:
  MoveItPy() → PlanningComponent("arm") → plan(target) → execute()

支持的规划目标:
  1. 关节空间目标 → plan(target=joints_dict)
  2. 位姿目标   → plan(target=Pose)     [需 IK]
  3. 笛卡尔路径  → compute_cartesian_path([waypoints])
  4. 命名位姿   → plan(target="home")
  5. 约束规划   → plan(constraints=my_constraints)
```

**初始化模板:**
```python
from moveit.planning import MoveItPy, PlanningComponent
moveit = MoveItPy(node_name='moveit_py')
arm = PlanningComponent(moveit, 'panda_arm')
arm.plan(target={'panda_joint1': 0.5, ...})
arm.execute(plan_result)
```

---

## 第4页: 正运动学 (FK)

**FK: 关节角 θ → 末端位姿 T**

```
DH 参数法:
  连杆 i: a_i, d_i, α_i, θ_i
  齐次变换: A_i = Rot_z(θ_i)·Trans_z(d_i)·Trans_x(a_i)·Rot_x(α_i)

末端相对于基座:
  T_0^n = A_1 · A_2 · A_3 · ... · A_n

Panda 7-DOF 运动学链:
  base → joint1 → joint2 → joint3 → joint4 → joint5 → joint6 → joint7 → EE
```

**Python FK 实现框架:**
```python
def dh_transform(a, d, alpha, theta):
    ct, st = cos(theta), sin(theta)
    ca, sa = cos(alpha), sin(alpha)
    return [[ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d   ],
            [0,   0,      0,     1   ]]

def forward_kinematics(joints, dh_params):
    T = eye(4)
    for j, (a, d, alpha, _) in zip(joints, dh_params):
        T = T @ dh_transform(a, d, alpha, j)
    return T
```

---

## 第5页: 逆运动学 (IK)

**IK: 末端位姿 T → 关节角 θ  (1 对多)**

```
求解方法:
  ┌─ 解析法 ── Pieper 判据: 3 个连续轴平行或交于一点
  │
  └─ 数值法 ── 牛顿-拉夫逊迭代
       Jacobian pseudoinverse: Δθ = J⁺ · Δx
       Damped Least Squares:  Δθ = Jᵀ(J·Jᵀ + λ²I)⁻¹·Δx

Panda 机械臂满足 Pieper 判据 → 解析解存在
  (后 3 个关节轴交于腕部中心点)
```

**IK 求解挑战:**
- 多解性: 通常有 8 组解 (elbow up/down × wrist up/down × shoulder up/down)
- 奇异点: 某些位姿下 Jacobian 行不满秩
- 自碰撞: 解可能触发自碰撞

---

## 第6页: 运动规划器对比

| 规划器 | 类型 | 速度 | 最优性 | 适用场景 |
|--------|------|------|--------|---------|
| RRTConnect | 采样 | ★★★★★ | ★☆☆☆☆ | 快速探索, 通用 |
| RRT* | 采样 | ★★★☆☆ | ★★★★☆ | 需路径质量 |
| PRM | 采样 | ★★★★☆ | ★★★☆☆ | 静态环境, 多次查询 |
| EST | 采样 | ★★★★☆ | ★☆☆☆☆ | 高维空间 |
| CHOMP | 优化 | ★★★☆☆ | ★★★★☆ | 平滑轨迹 |
| STOMP | 优化 | ★★☆☆☆ | ★★★★☆ | 无需梯度 |
| TRRT | 采样 | ★★☆☆☆ | ★★★☆☆ | 狭窄通道 |

---

## 第7页: 规划场景 (Planning Scene)

```
Planning Scene = 机器人 + 环境

  ┌─────────────────────────┐
  │  Robot Model             │
  │  ├── URDF 运动学链       │
  │  ├── SRDF 碰撞矩阵       │
  │  └── 关节限制            │
  ├─────────────────────────┤
  │  World Objects           │
  │  ├── CollisionObject: 障碍│
  │  ├── AttachedObject: 附着│
  │  └── Octomap: 点云地图   │
  ├─────────────────────────┤
  │  Planning Constraints    │
  │  ├── 位姿约束            │
  │  ├── 关节约束            │
  │  └── 可见性约束          │
  └─────────────────────────┘
```

**ACL (Allowed Collision Matrix):**
```
        j1  j2  j3  j4  j5  j6  j7
    j1   X   ✓   ✓   ✓   ✓   ✓   ✓   ✓ = 允许碰撞
    j2   ✓   X   ✓   ✓   ✓   ✓   ✓   X = 不允许
    j3   ...                           (相邻连杆间默认不允许)
```

---

## 第8页: 碰撞检测流程

```
对每个规划请求:
  1. 载入当前 RobotState
  2. 对轨迹上的每个时间戳:
     a) 正运动学计算所有连杆姿态
     b) 对每个碰撞对 (ACL 不允许的):
        - 计算 FCL/Bullet 最近距离
        - 若距离 < min_distance → 标记为碰撞
  3. 将碰撞状态反馈给规划器
  4. 规划器丢弃碰撞采样/轨迹
```

**FCL (Flexible Collision Library):**
```
支持的形状: Box, Sphere, Cylinder, Cone, Mesh
碰撞检测:   AABB 粗检测 → 精确几何检测
距离计算:   计算两形状间最小距离
```

---

## 第9页: 抓取规划 (Grasp Planning)

```
抓取任务分解:
  1. 目标识别    ── 视觉检测目标物体
  2. 候选生成    ── 生成可能的抓取位姿
  3. 抓取排序    ── 按质量/成功率排序
  4. IK 求解     ── 逆运动学验证可达性
  5. 规划执行    ── 运动规划到抓取位姿

Grasp 消息: moveit_msgs/Grasp
  ├── grasp_pose:     夹爪中心位姿
  ├── pre_grasp_approach: 接近方向
  ├── post_grasp_retreat: 撤退方向
  ├── grasp_quality:  抓取质量 (0~1)
  └── max_contact_force: 最大接触力
```

**抓取策略:**
```
顶部抓取 (Top Grasp):
  ┌─────────┐
  │   ╔═╗   │  ← 夹爪从上方接近
  │   ║O║   │
  │   ╚═╝   │
  └─────────┘

侧面抓取 (Side Grasp):
  ┌─────────┐
  │  ╔═╗    │
  │  ║O║    │  ← 夹爪从侧面接近
  │  ╚═╝    │
  └─────────┘
```

---

## 第10页: ros2_control 集成

```
MoveIt 2 通过 ros2_control 驱动真实/仿真硬件:

  MoveIt 2 ──[trajectory]──> joint_trajectory_controller
                                    │
                          ┌─────────▼─────────┐
                          │  ros2_control     │
                          │  ├── 硬件接口      │
                          │  ├── 资源管理器    │
                          │  └── 控制器管理器  │
                          └─────────┬─────────┘
                                    │
                          ┌─────────▼─────────┐
                          │  执行器 (真实/仿真)│
                          └───────────────────┘
```

**控制器切换:**
```bash
# 列出当前控制器
ros2 control list_controllers

# 激活位置控制器 (MoveIt 需要)
ros2 control switch_controllers --activate joint_trajectory_controller
```

---

## 第11页: MoveIt 2 配置文件

```
robot_moveit_config/
├── config/
│   ├── robot.srdf           ← 语义描述 (运动组、碰撞矩阵、位姿)
│   ├── robot.urdf           ← 机器人描述
│   ├── joint_limits.yaml    ← 关节速度/加速度限制
│   ├── kinematics.yaml      ← IK 求解器配置
│   ├── ompl_planning.yaml   ← OMPL 规划器配置
│   ├── chomp_planning.yaml  ← CHOMP 配置
│   ├── moveit_controllers.yaml ← 控制器映射
│   ├── ros2_controllers.yaml   ← ros2_control 配置
│   └── sensors_3d.yaml      ← 3D 传感器配置
├── launch/
│   ├── move_group.launch.py
│   ├── demo.launch.py
│   ├── moveit_rviz.launch.py
│   └── setup_assistant.launch.py
└── rviz/
    └── moveit.rviz
```

---

## 第12页: 本章总结

**核心要点回顾:**
1. MoveIt 2 提供机械臂运动规划的完整工具链
2. MoveItPy 封装 Python API, 支持关节/位姿/笛卡尔规划
3. 运动规划器 (OMPL/CHOMP/STOMP) 各有适用场景
4. 规划场景维护完整环境模型, 支持碰撞检测
5. ros2_control 连接规划与执行

**关键命令:**
```bash
ros2 launch moveit2 moveit.launch.py        # 启动 MoveIt
ros2 run moveit_setup_assistant setup_assistant  # 配置向导
ros2 control list_controllers               # 查看控制器
```

**下一步: 第13章 YOLO + ROS 2 目标检测**