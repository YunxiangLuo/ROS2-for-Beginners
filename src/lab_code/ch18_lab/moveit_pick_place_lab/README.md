# moveit_pick_place_lab — 第 18 章 MoveIt 抓取放置实验

- 包类型：`ament_python`
- ROS 2 Jazzy + MoveIt 2 + xArm6（`xarm_ros2_arm_only`）

## 简介

本章练习 MoveIt 抓取-放置全流程。各演示的规范实现由 `course_lab_utils`

提供，本包以 console scripts 转发：

| 程序 | 内容 |
|------|------|
| `target_publisher` | 发布 3D 目标位姿（供抓取定位） |
| `pick_place_demo` | 抓取-放置全流程 |
| `obstacles_demo` | 带障碍物的避障规划 |
| `beeline_demo` | 直线路径规划 |
| `attach_object_demo` | 物体附着/分离（AttachedCollisionObject） |

## 构建

```bash

cd <robot_sim_demo 工作区>

source /opt/ros/jazzy/setup.bash
# 需先构建并 source xarm_description 底层与 course_lab_utils
colcon build --symlink-install --packages-select moveit_pick_place_lab

source install/setup.bash
```

## 运行

先启动 xArm 仿真（含 MoveIt）：

```bash
ros2 launch xarm_ros2_arm_only arm_only.launch.py
```

再运行演示（另开终端）：

```bash

ros2 run moveit_pick_place_lab target_publisher   # 先发布目标

ros2 run moveit_pick_place_lab pick_place_demo

ros2 run moveit_pick_place_lab obstacles_demo

ros2 run moveit_pick_place_lab beeline_demo

ros2 run moveit_pick_place_lab attach_object_demo
```

## 测试

```bash
colcon test --packages-select moveit_pick_place_lab
colcon test-result --all
```

## 运行结果

`pick_place_demo` 依次完成接近→抓取→提升→移动→放置→归位，终端输出各

阶段规划结果；Gazebo/RViz 中可见目标物体被拾起与放下。截图保存至

`docs/images/moveit_pick_place.png`。
