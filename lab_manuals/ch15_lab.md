# 第15章 实验：机械臂基础与关节空间控制

## 当前仓库仿真验证：xArm 关节状态发布与 RViz 可视化

### 实验目标

用四个 Python 发布器向 `/joint_states` 发布 xArm 的 8 个关节角（`arm_1_joint`~`arm_6_joint`、`gripper_1_joint`、`gripper_2_joint`），配合 `robot_state_publisher` 在 RViz 中观察机械臂往复运动与夹爪开合。xArm 仿真环境由仓库 `src/xarm/` 的 arm_only 一体化 launch 提供。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 终端 1：xArm 一体化仿真（MoveIt + RViz，含 robot_state_publisher）
ros2 launch xarm_ros2_arm_only arm_only_move_group.launch.py
```

```bash
# 终端 2：发布关节状态（四选一）
ros2 run arm_joint_pub_lab arm_joints_pub1
# ros2 run arm_joint_pub_lab arm_gripper
# ros2 run arm_joint_pub_lab gripper_open_close
# ros2 run arm_joint_pub_lab hello_arm_node
```

```bash
# 终端 3：验证话题
ros2 topic echo /joint_states --once
```

### 观察与验收

RViz 中 xArm 模型跟随关节角运动：`arm_joints_pub1` 令 `arm_2_joint` 在 ±1.5rad 内往复摆动；`gripper_open_close` 令夹爪两指同步开合（0~0.65rad）；`ros2 topic echo` 输出的 8 个关节名必须与 xArm URDF 一致，`position`/`velocity`/`effort` 数组与 `name` 等长。源码边界：`src/lab_code/ch15_lab/arm_joint_pub_lab/`、`src/xarm/`。

## 实际运行证据

本实验为纯 Python 发布器教学（不依赖真实机械臂硬件），运行证据需上机自采：记录 `ros2 topic echo /joint_states --once` 的输出，并将 RViz 中机械臂往复运动的截图保存至 `docs/images/arm_joint_pub.png`（包 README 约定）。仓库 `src/xarm/` 的 arm_only 一体化仿真提供同名关节的真实运行环境，其端到端冒烟检查脚本 `arm_only_runtime_smoke` 会校验三个活动控制器与 8 个关节命名。

> **对应理论章节**：第24章《机械臂基础知识》
> **实验课时**：2课时  
> **实验代码**：`src/lab_code/ch15_lab/`（`arm_joint_pub_lab/` 功能包：四个关节状态发布器）  

## 实验目标
- 掌握 `sensor_msgs/JointState` 消息结构与发布方法
- 理解机械臂关节命名与 `robot_state_publisher` 的配合机制
- 会用定时器周期发布实现关节往复运动
- 理解关节空间控制的基本形式：直接指定各关节目标角
- 了解夹爪（平动指）开合控制与机械臂联动

## 实验环境
- ROS 2 Jazzy
- xArm 一体化仿真（`src/xarm/xarm_ros2_arm_only`）
- RViz2
- `sensor_msgs/msg/JointState`（机器人标准关节接口）

## 参考代码说明
`src/lab_code/ch15_lab/arm_joint_pub_lab/` 为 ament_python 功能包，`setup.py` 注册了 4 个 `console_scripts` 入口：

```
src/lab_code/ch15_lab/
├── arm_joint_pub_lab/                  # ROS2 功能包（ament_python）
│   ├── arm_joint_pub_lab/
│   │   ├── hello_arm_node.py           # 通用教学示例（joint1/finger 命名）
│   │   ├── arm_joints_pub1.py          # xArm 8 关节，arm_2_joint 往复
│   │   ├── arm_gripper.py              # 臂手联动循环
│   │   └── gripper_open_close.py       # 夹爪开合
│   └── setup.py / package.xml
└── README.md
```

四个发布器的行为对照：

| 发布器 | 周期 | 行为 |
|------|------|------|
| `hello_arm_node` | 0.1s | 教学命名（`joint1`~`joint3`、`finger1/2_joint`）：`joint2` ±1.5rad 往复、`joint3=0.5×joint2`、夹爪同步开合，并打印关节角日志 |
| `arm_joints_pub1` | 0.05s | xArm 8 关节；`arm_2_joint` 以 0.015 步长在 ±1.5rad 间往复 |
| `arm_gripper` | 0.05s | 以 `cycle`（0~100 / 100~200）控制 `arm_1_joint`、`arm_4_joint` 与两指关节正反交替 |
| `gripper_open_close` | 0.05s | `gripper_1_joint`/`gripper_2_joint` 同步在 0~0.65rad 间开合 |

- 所有发布器都只写 `JointState` 的 `name` 与 `position`，由 `robot_state_publisher` 将关节角映射为模型位姿与 TF——这是关节空间控制的最低层接口，第26章起 MoveIt2 的规划结果最终也通过 `/joint_states` 驱动模型。
- 关节名与 `src/xarm` 的 xArm URDF 完全一致（6 个旋转关节 + 2 个平行夹爪指关节），因此无需改任何参数即可在 arm_only 仿真中展示。

## 实验步骤

1. 构建实验包：

```bash
cd <机器人工作区>       # 即含 src/lab_code 的 ROS 2 工作区
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select arm_joint_pub_lab
source install/setup.bash
```

2. 终端 1 启动 xArm 一体化仿真：

```bash
ros2 launch xarm_ros2_arm_only arm_only_move_group.launch.py
```

3. 终端 2 依次运行四个发布器（每次只运行一个，对比行为差异）。

4. 终端 3 验证关节数据：

```bash
ros2 topic echo /joint_states --once
```

5. 在 RViz 中观察：`arm_joints_pub1` 的 2 号关节往复、`gripper_open_close` 的夹爪开合；再用「关节名 → 数值」对照 URDF 的关节定义。

6. （可选）修改 `arm_joints_pub1.py` 的步长（0.015）与幅值（±1.5），重新 `python3` 运行（或构建后重跑），观察往复速度变化。

7. 实验结束按 Ctrl+C 停止所有进程。

## 实验结果与分析

- `ros2 topic echo` 显示 `arm_2_joint` 的 position 随时间呈三角波往复（步长 0.015、范围 ±1.5rad），说明定时器 + 状态翻转即可实现连续关节运动。
- 夹爪两指关节数值同步增/减（0~0.65rad），与 URDF 中对称的指连杆定义对应，体现第24章所述平行夹爪的自由度结构。
- `robot_state_publisher` 以 `/joint_states` 为输入发布关节间 TF 与模型显示，验证了「关节空间控制只需给定关节角、无需逆运动学」的基本结论。

## 思考题

1. `JointState` 的 `name`/`position`/`velocity`/`effort` 数组为什么要等长对齐？`name` 顺序变化会影响什么？
2. 为什么关节空间控制只需指定各关节角，而不需要求解逆运动学？
3. `arm_gripper` 的 `cycle` 计数实现的是什么行为？与 `gripper_open_close` 的边界翻转写法有何异同？
4. 若把 `hello_arm_node` 的 `joint2` 步长从 0.02 改为 0.2，其角度-时间曲线会有什么变化？
5. 本章的 `/joint_states` 发布与第16章 `robot_state_publisher` 的关系是什么？在完整机械臂系统中谁负责真实关节状态的读取与发布？