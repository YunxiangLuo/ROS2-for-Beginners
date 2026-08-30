# 第14章 实验：Nav2 一键导航综合实训

## 当前仓库仿真验证：Gazebo 仿真环境一键 Nav2 导航

### 实验目标

一键启动 Gazebo（Wheeltec + ISCAS Museum）、Nav2 导航栈与 RViz，再用 `nav_goal_runner` 依次发送 4 个目标点完成自动巡航，观察 AMCL 粒子云、全局/局部路径与 Nav2 生命周期节点状态。本章与第13章同属第23章综合实训的「先建图、后导航」流程。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 终端 1：一键启动 Gazebo + Nav2 + RViz
ros2 launch nav_bringup_lab nav_bringup.launch.py
# 无 GUI 环境：ros2 launch nav_bringup_lab nav_bringup.launch.py gui:=false use_rviz:=false
```

```bash
# 终端 2：自动巡航（模拟时间必须同步）
ros2 run nav_bringup_lab nav_goal_runner --ros-args -p use_sim_time:=true
```

### 观察与验收

RViz 中依次出现地图、粒子云、全局路径与局部路径，机器人自主行驶到各目标点；`nav_goal_runner` 实时打印「距目标: x.xx m」并在到达后输出耗时与行驶距离，最终汇总「导航完成: 4/4 个目标成功到达」。`/amcl`、`/bt_navigator`、`/planner_server`、`/controller_server` 生命周期状态均为 active。源码边界：`src/lab_code/ch14_lab/`、`src/navigation_sim_demo_ros2/launch/nav2_demo.launch.py`（默认地图 `Software_Museum.yaml`）。

> 注意：本实验启动前先停止第13章 SLAM 的全部进程——两套 launch（建图/导航）都会启动 Gazebo 与同名节点，同时运行会造成话题冲突。

## 实际运行证据

真实运行的组合 launch、多点自动巡航、生命周期检查与代价地图输出：

![ch14 Nav2 一键导航运行输出](images/runtime/nonlab_nav2.gif)

原始录制：[nonlab_nav2.cast](images/runtime/nonlab_nav2.cast)。

> **对应理论章节**：第23章《SLAM与导航综合实训》（Nav2 一键导航部分）
> **实验课时**：2课时  
> **实验代码**：`src/lab_code/ch14_lab/`（`nav_bringup_lab/` 功能包 + `nav2_bringup.sh` + `nav_goal_runner.py`）  

## 实验目标
- 掌握一键 Nav2 导航的组成：仿真环境 + Nav2 栈 + RViz
- 理解 AMCL 定位、全局/局部代价地图与行为树在导航中的角色
- 会用 `nav_goal_runner` 发送多点导航目标并解读反馈
- 会用生命周期工具检查 Nav2 节点状态
- 理解地图在 SLAM 与导航之间的衔接（第13章建图 → 本章导航）

## 实验环境
- ROS 2 Jazzy + Gazebo Sim Harmonic + Nav2
- `navigation_sim_demo_ros2`（Nav2 栈，默认地图 `Software_Museum.yaml`）
- `robot_sim_demo`（Wheeltec + ISCAS Museum）
- RViz2、nav2_simple_commander

## 参考代码说明
`src/lab_code/ch14_lab/` 下包含一个 ament_python 功能包、一个交互式启动脚本与一个顶层源码：

```
src/lab_code/ch14_lab/
├── nav_bringup_lab/                        # ROS2 功能包（ament_python）
│   ├── launch/nav_bringup.launch.py        # 组合启动 Gazebo + Nav2 + RViz
│   ├── nav_bringup_lab/nav_goal_runner.py  # 导航目标发送（转发实现）
│   └── package.xml / setup.py
├── nav2_bringup.sh                         # 交互式启动脚本
├── nav_goal_runner.py                      # 导航目标发送（顶层完整实现）
└── README.md
```

- `nav_bringup.launch.py` 包含两个 Include：`robot_sim_demo/gazebo2.launch.py`（`gui` 透传、`rviz:=false`、`drive:=false`）与 `navigation_sim_demo_ros2/nav2_demo.launch.py`（`use_gazebo:=false`、`use_rviz` 透传、`use_sim_time:=true`），地图固定为 `Software_Museum.yaml`。
- 顶层 `nav_goal_runner.py` 为完整实现：`BasicNavigator` 设置初始位姿（5.0, 0.0, -2.0）并 `waitUntilNav2Active()`；依次向 `navigate_to_pose` 动作服务器发送 4 个目标点 (3.0,-1.0,-2.0)、(2.0,1.5,1.57)、(5.0,0.5,0.0)、(1.0,-0.5,3.14)；期间打印 `distance_remaining` 反馈，到达后输出耗时与行驶距离，失败则跳过剩余目标；`use_sim_time` 从节点参数读取并同步到导航器。
- `nav2_bringup.sh` 分 5 步：检查 ROS 2 环境 → 检查 `~/maps/office_map.yaml`（存在则用 `map:=` 覆盖，否则回退默认地图）→ 启动 `gazebo2.launch.py gui:=true rviz:=false drive:=false` → 8s 后启动 `nav2_demo.launch.py` → 5s 后对 `/amcl`、`/bt_navigator`、`/planner_server`、`/controller_server` 做生命周期检查，并提示 RViz（`navigation.rviz`）、手动目标、代价地图与 AMCL 话题命令。

## 实验步骤

1. 构建实验包：

```bash
cd <机器人工作区>       # 即含 src/lab_code 的 ROS 2 工作区
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select nav_bringup_lab
source install/setup.bash
```

2. （可选）把第13章保存的地图放到默认位置，`nav2_bringup.sh` 会自动发现并覆盖默认地图：

```bash
mkdir -p ~/maps
cp <第13章输出的 office_map.yaml、office_map.pgm> ~/maps/
```

3. 一键启动导航（二选一）：

```bash
# Launch 方式（推荐，使用默认 Software_Museum 地图）
ros2 launch nav_bringup_lab nav_bringup.launch.py

# Shell 脚本方式（自动探测 office_map.yaml / 回退默认地图）
bash src/lab_code/ch14_lab/nav2_bringup.sh
```

4. 另开终端运行自动巡航：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run nav_bringup_lab nav_goal_runner --ros-args -p use_sim_time:=true
```

5. 观察与巡检：

```bash
ros2 lifecycle get /amcl
ros2 lifecycle get /bt_navigator
ros2 topic echo /plan --once            # 全局路径
ros2 topic echo /amcl_pose --once       # AMCL 定位结果
```

6. （可选）用 RViz 的 "Nav2 Goal" 按钮手动下发目标，或在终端用 2D Goal Estimate 纠正初始位姿。

7. 实验结束按 Ctrl+C 停止所有进程。

## 实验结果与分析

- 巡航过程中「距目标: x.xx m」的 `distance_remaining` 反馈单调递减至 0；每个目标到达后打印耗时与行驶距离，说明局部规划（DWA）持续修正机器人朝向并最终收敛。
- `/amcl` 等 4 个节点生命周期为 active，说明 Nav2 调用图（BT NavigateToPose）已激活，导航动作才能被接受。
- 若第 2 步放置了 `office_map.yaml`，`nav2_bringup.sh` 会打印「使用地图: ~/maps/office_map.yaml」——这就完成了「第13章建图 → 本章装载定位导航」的完整闭环。
- AMCL 粒子云从初始位姿（5.0, 0.0, -2.0）附近逐渐收敛，`/amcl_pose` 反映收敛后的定位结果。

## 思考题

1. 若目标点落在未知区域或被障碍物完全挡住，`nav_goal_runner` 会打印什么？Nav2 全局规划器如何表现？
2. AMCL 的初始位姿为什么重要？`nav_goal_runner` 如何设置它？设置错误会有什么后果？
3. 为什么运行 `nav_goal_runner` 必须加 `-p use_sim_time:=true`？（提示：与第22章仿真时钟的关系）
4. 为什么必须先停掉第13章 SLAM 的进程再启动本章导航？两套流程共用哪些节点名与话题？