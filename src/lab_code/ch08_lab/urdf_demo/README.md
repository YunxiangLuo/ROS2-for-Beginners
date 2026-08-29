# urdf_demo

第 8 章实验包：URDF/Xacro 机器人建模练习。

- 包类型：`ament_python`
- ROS 2 Jazzy
- 依赖：需安装 `joint_state_publisher_gui`

## 简介

本包演示如何使用 URDF / Xacro 描述机器人模型，并通过 launch 文件在 RViz 中可视化。本包无可执行节点。

## Launch 文件

| 文件 | 功能 |
| --- | --- |
| `display.launch.py` | 启动 `robot_state_publisher` + `joint_state_publisher_gui` + `rviz2`，加载 `urdf/simple_robot.xacro` |

## 构建命令

```bash
# 安装依赖（若未安装）
sudo apt install ros-jazzy-joint-state-publisher-gui

cd <workspace>

colcon build --symlink-install --packages-select urdf_demo

source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

## 运行步骤

```bash
ros2 launch urdf_demo display.launch.py
```

启动后 RViz 中将显示机器人模型，可通过 `joint_state_publisher_gui` 调节关节角度。
