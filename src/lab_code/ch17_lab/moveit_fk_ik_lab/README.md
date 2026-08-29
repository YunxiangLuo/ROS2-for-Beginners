# moveit_fk_ik_lab — 第 17 章 MoveIt FK/IK 实验

- 包类型：`ament_python`
- ROS 2 Jazzy + MoveIt 2 (MoveItPy) + xArm6（`xarm_ros2_arm_only`）

## 简介

本章练习用 MoveItPy API 进行正运动学（关节空间）与逆运动学（位姿空间）

规划。所有程序面向 xArm 的 `xarm` 规划组，末端为 `gripper_centor_link`，

规划辅助函数（`set_joint_goal`/`set_pose_goal`/`plan_and_execute` 等）由

`course_lab_utils` 提供。

| 程序 | 内容 |
|------|------|
| `fk_demo` | 关节空间目标 + 夹爪开合 + Home 归位 |
| `ik_demo` | 位姿目标 IK 规划 |
| `fk_ik_exercise` | FK/IK 综合练习（完成版） |
| `rectangle_exercise` | 笛卡尔路径画矩形练习（完成版） |

## 构建

```bash

cd <robot_sim_demo 工作区>

source /opt/ros/jazzy/setup.bash
# 需先构建并 source xarm_description 底层与 course_lab_utils
colcon build --symlink-install --packages-select moveit_fk_ik_lab

source install/setup.bash
```

## 运行

先启动 xArm 仿真（含 MoveIt）：

```bash
ros2 launch xarm_ros2_arm_only arm_only.launch.py
```

再运行实验程序（另开终端）：

```bash

ros2 run moveit_fk_ik_lab fk_demo

ros2 run moveit_fk_ik_lab ik_demo

ros2 run moveit_fk_ik_lab fk_ik_exercise

ros2 run moveit_fk_ik_lab rectangle_exercise
```

## 测试

```bash
colcon test --packages-select moveit_fk_ik_lab
colcon test-result --all
```

## 运行结果

程序依次输出各目标规划与执行日志（如 `arm joints: [...]`、`pose x=0.30`），

Gazebo/RViz 中机械臂按规划运动并最终回到 Home。截图保存至

`docs/images/moveit_fk_ik.png`。
