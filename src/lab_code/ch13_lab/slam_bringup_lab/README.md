# slam_bringup_lab — 第 13 章 SLAM 建图实验

- 包类型：`ament_python`
- ROS 2 Jazzy + Gazebo Sim Harmonic

## 简介

本章练习 SLAM 在线建图。移动机器人仿真统一使用 `robot_sim_demo`
（Wheeltec + ISCAS Museum），SLAM 由 `slam_sim_demo_ros2`（slam_toolbox）
提供。`slam_bringup_lab` 的 launch 一键启动两者；`slam_map_runner` 转发
`slam_sim_demo_ros2` 的自动建图监控实现。

> 章节根目录的 `slam_bringup.sh` 负责先启动 `robot_sim_demo`，再启动
> `slam_sim_demo_ros2`；也可以直接使用本包的组合 launch。

## 构建

```bash
cd <robot_sim_demo 工作区>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select slam_bringup_lab
source install/setup.bash
```

## 运行

```bash
# 一键启动 Gazebo + SLAM + RViz
ros2 launch slam_bringup_lab slam_bringup.launch.py

# 无 GUI 模式
ros2 launch slam_bringup_lab slam_bringup.launch.py gui:=false use_rviz:=false

# 自动建图（另开终端）
ros2 run slam_bringup_lab slam_map_runner
```

启动后检查：

```bash
ros2 topic echo /map --field info --once   # 地图信息
ros2 topic echo /scan --once               # 激光扫描
ros2 run tf2_tools view_frames             # TF 树: map→odom→base_link→laser_link
```

## 测试

```bash
colcon test --packages-select slam_bringup_lab
colcon test-result --all
```

## 运行结果

RViz 中可见 `/map` 随机器人运动逐渐扩展，扫描点云与墙体对齐；
`slam_map_runner` 终端输出地图覆盖率。截图保存至
`docs/images/slam_mapping.png`。
