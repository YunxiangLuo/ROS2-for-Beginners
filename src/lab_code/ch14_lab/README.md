# 第14章 实验代码：Nav2 一键导航综合实训

本章学习使用 ROS2 Nav2 实现机器人的自主导航。

## 文件说明

| 文件 | 用途 | 运行方式 |
|------|------|----------|
| `nav2_bringup.sh` | Nav2 导航启动脚本。启动机器人驱动、激光雷达、Nav2 导航堆栈和 RViz | `bash nav2_bringup.sh` |
| `nav_goal_runner.py` | 自动导航目标点发布节点。向 Nav2 发送一系列目标点，实现自动巡航 | `python3 nav_goal_runner.py` |

## 运行说明

### 步骤1：启动导航

```bash
cd src/lab_code/ch14_lab/
bash nav2_bringup.sh
```

### 步骤2：运行自动巡航

在新终端中运行：

```bash
cd src/lab_code/ch14_lab/
python3 nav_goal_runner.py
```

### 手动发送目标

也可以通过 RViz2 的 "2D Goal Pose" 按钮手动设置导航目标，或使用命令行：

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}}}"
```
