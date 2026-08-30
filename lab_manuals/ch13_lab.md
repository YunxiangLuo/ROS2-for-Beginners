# 第13章 实验：SLAM 一键建图综合实训

## 当前仓库仿真验证：Gazebo 仿真环境一键 SLAM 建图

### 实验目标

一键启动 Gazebo（Wheeltec + ISCAS Museum）、slam_toolbox 在线建图与 RViz，再用 `slam_map_runner` 自动遍历环境，观察 `/map` 的实时生长过程与建图质量指标（已知栅格、占据率、探索率）。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 终端 1：一键启动 Gazebo + SLAM + RViz
ros2 launch slam_bringup_lab slam_bringup.launch.py
# 无 GUI 环境：ros2 launch slam_bringup_lab slam_bringup.launch.py gui:=false use_rviz:=false
```

```bash
# 终端 2：自动遍历建图
ros2 run slam_bringup_lab slam_map_runner
```

### 观察与验收

RViz 中 `/map` 随机器人运动逐渐扩展，扫描点云与墙体对齐；`slam_map_runner` 每 5 次地图更新打印一次「已知 / 占据 / 空闲 / 探索率」，达到完成条件后机器人自动停止并输出 `slam-map-updated`。TF 树应为 `map→odom→base_link→laser_link`。源码边界：`src/lab_code/ch13_lab/`、`src/slam_sim_demo_ros2/launch/slam_demo.launch.py`。

## 实际运行证据

真实运行的组合 launch、自动遍历建图与地图保存输出：

![ch13 SLAM 一键建图运行输出](images/runtime/nonlab_slam.gif)

原始录制：[nonlab_slam.cast](images/runtime/nonlab_slam.cast)。

> **对应理论章节**：第23章《SLAM与导航综合实训》（SLAM 一键建图部分）
> **实验课时**：2课时  
> **实验代码**：`src/lab_code/ch13_lab/`（`slam_bringup_lab/` 功能包 + `slam_bringup.sh` + `slam_map_runner.py`）  

## 实验目标
- 掌握一键 SLAM 建图的组成：仿真环境 + 激光雷达 + slam_toolbox + RViz
- 理解 launch 参数 `gui` / `use_rviz` / `drive` 的作用
- 会用自动遍历节点 `slam_map_runner` 驱动机器人完成建图
- 能解读建图质量指标：已知栅格、占据率、探索率
- 学会保存地图供后续导航实验使用

## 实验环境
- ROS 2 Jazzy + Gazebo Sim Harmonic
- slam_toolbox（`slam_sim_demo_ros2`）
- `robot_sim_demo`（Wheeltec + ISCAS Museum）
- RViz2、tf2_tools、nav2_map_server

## 参考代码说明
`src/lab_code/ch13_lab/` 下包含一个 ament_python 功能包、一个交互式启动脚本与一个顶层源码：

```
src/lab_code/ch13_lab/
├── slam_bringup_lab/                       # ROS2 功能包（ament_python）
│   ├── launch/slam_bringup.launch.py       # 组合启动 Gazebo + SLAM + RViz
│   ├── slam_bringup_lab/slam_map_runner.py # 自动建图监控节点（9 行转发器）
│   └── package.xml / setup.py
├── slam_bringup.sh                         # 交互式启动脚本
├── slam_map_runner.py                      # 自动建图监控节点（顶层完整实现）
└── README.md
```

- `slam_bringup.launch.py` 包含两个 Include：`robot_sim_demo/gazebo2.launch.py`（`gui` 透传、`rviz:=false`、`drive:=false`）与 `slam_sim_demo_ros2/slam_demo.launch.py`（`use_gazebo:=false`、`use_rviz` 透传、`use_sim_time:=true`）。
- 顶层 `slam_map_runner.py` 为完整实现：节点 `slam_map_runner` 订阅 `/odom`、`/map`，发布 `/cmd_vel`；以 8 秒为周期运动（前 2s 左转 0.6rad/s，2~4.5s 前进 0.25m/s，4.5~6.5s 右转，其后前进）；每 5 次地图更新打印一次栅格统计与探索率；满足「地图更新≥2 次、行驶距离>0.2m、已知栅格增长>100」即停止并输出 `slam-map-updated`。
- `slam_bringup.sh` 分 4 步：检查 ROS 2 环境 → 启动 `gazebo2.launch.py gui:=true rviz:=false drive:=false` → 等 10s 后启动 `slam_demo.launch.py` → 提示键盘遥控、RViz 与保存地图命令。

## 实验步骤

1. 构建实验包：

```bash
cd <机器人工作区>       # 即含 src/lab_code 的 ROS 2 工作区
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select slam_bringup_lab
source install/setup.bash
```

2. 一键启动建图（二选一）：

```bash
# Launch 方式（推荐）
ros2 launch slam_bringup_lab slam_bringup.launch.py

# Shell 脚本方式
bash src/lab_code/ch13_lab/slam_bringup.sh
```

3. 另开终端运行自动遍历节点：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run slam_bringup_lab slam_map_runner
```

4. 观察 RViz 中 `/map` 的扩展过程与 runner 终端的栅格统计；在「1. 左转 / 2. 前进 / 3. 右转 / 4. 前进」的周期性运动中，观察地图在前向运动时增长最快。

5. （可选）对遍历遗漏的区域，用键盘遥控补充：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

6. 保存地图（第14章导航实验直接使用）：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/office_map
```

7. 实验结束按 Ctrl+C 停止所有进程。

## 实验结果与分析

- runner 每 5 次地图更新打印一次统计，例如「已知: 58200, 占据: 8832(15.2%), 空闲: 43819, 探索率: 46.2%」：探索率 = 已知栅格数 / 地图总栅格数，占据率 = 占据栅格数 / 已知栅格数，两者分别反映环境覆盖程度与墙体占比——与第10章占据栅格地图的概念一一对应。
- TF 树 `map→odom→base_link→laser_link` 说明里程计与激光雷达外参正确接入，是 slam_toolbox 能正确建图的前提。
- 保存的 `office_map.yaml`/`office_map.pgm` 将在第14章 Nav2 导航实验中被 `nav2_demo.launch.py` 直接加载。

## 思考题

1. `slam_demo.launch.py` 为什么设置 `use_gazebo:=false`？如果另行用 `drive:=true` 的 `gazebo2.launch.py` 启动第二套仿真，两套 `/scan`、`/odom` 会如何？
2. 探索率的两种变化趋势：机器人停在原地 vs 原地自转，探索率分别如何变化？为什么？
3. 占据率偏高（如 >30%）通常说明什么问题？
4. 自动遍历的 8 秒周期运动能否保证全覆盖？与手动键盘建图相比优劣各是什么？