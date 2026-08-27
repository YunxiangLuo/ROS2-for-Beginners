# nav_bringup_lab — 第 14 章 Nav2 导航实验

- 包类型：`ament_python`
- ROS 2 Jazzy + Gazebo Sim Harmonic + Nav2

## 简介

本章练习 Nav2 自主导航。移动机器人仿真统一使用 `robot_sim_demo`
（Wheeltec + ISCAS Museum），Nav2 栈由 `navigation_sim_demo_ros2` 提供，
地图为 `Software_Museum.yaml`。`nav_bringup_lab` 的 launch 一键启动两者；
`nav_goal_runner` 转发 `navigation_sim_demo_ros2` 的目标发送实现。

> 章节根目录的 `nav2_bringup.sh` 负责先启动 `robot_sim_demo`，再启动
> `navigation_sim_demo_ros2`；也可以直接使用本包的组合 launch。

## 构建

```bash
cd <robot_sim_demo 工作区>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select nav_bringup_lab
source install/setup.bash
```

## 运行

```bash
# 一键启动 Gazebo + Nav2 + RViz
ros2 launch nav_bringup_lab nav_bringup.launch.py

# 无 GUI 模式
ros2 launch nav_bringup_lab nav_bringup.launch.py gui:=false use_rviz:=false

# 发送导航目标（另开终端）
ros2 run nav_bringup_lab nav_goal_runner --ros-args -p use_sim_time:=true
```

启动后检查：

```bash
ros2 lifecycle get /amcl
ros2 lifecycle get /bt_navigator
ros2 topic echo /plan --once          # 全局路径
ros2 topic echo /cmd_vel --once       # 速度命令
```

## 测试

```bash
colcon test --packages-select nav_bringup_lab
colcon test-result --all
```

## 运行结果

RViz 中可见地图、粒子云、全局/局部路径；机器人自主行驶至目标点。
截图保存至 `docs/images/nav2_navigation.png`。
