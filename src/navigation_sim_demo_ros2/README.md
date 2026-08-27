# navigation_sim_demo_ros2

Nav2 导航仿真演示包：基于 Nav2 栈在 ISCAS Museum 仿真场景中实现自主导航。

## 依赖

- `robot_sim_demo`（Gazebo 仿真环境）
- Nav2 全栈（`nav2_bringup`, `nav2_map_server`, `nav2_amcl`, `nav2_controller`, `nav2_planner`, `nav2_bt_navigator` 等）
- `slam_toolbox`（可选，用于建图）

```bash
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander
```

## 构建

```bash
cd robot_sim_demo
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select navigation_sim_demo_ros2
source install/setup.bash
```

## 运行

### 方式一：独立启动 Gazebo + Nav2

```bash
# 终端 1：启动 Gazebo 仿真（不自动巡航，避免与 Nav2 冲突）
ros2 launch robot_sim_demo gazebo2.launch.py gui:=false rviz:=false drive:=false

# 终端 2：启动 Nav2 栈（不重复启动 Gazebo）
ros2 launch navigation_sim_demo_ros2 nav2_demo.launch.py \
  use_gazebo:=false use_rviz:=false use_sim_time:=true

# 终端 3：发送导航目标
ros2 run navigation_sim_demo_ros2 nav_goal_runner \
  --ros-args -p use_sim_time:=true -p goal_x:=1.0 -p goal_y:=0.0
```

### 方式二：一键启动 Gazebo + Nav2

```bash
ros2 launch navigation_sim_demo_ros2 nav2_demo.launch.py \
  use_gazebo:=true use_rviz:=false
```

### 方式三：带 RViz

```bash
ros2 launch navigation_sim_demo_ros2 nav2_demo.launch.py \
  use_gazebo:=true use_rviz:=true
```

## Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_gazebo` | `false` | 是否同时启动 Gazebo |
| `gz_headless` | `true` | Gazebo 无头模式 |
| `use_rviz` | `true` | 启动 RViz2 |
| `map` | `Software_Museum.yaml` | 地图文件 |
| `params_file` | `nav2_params.yaml` | Nav2 参数文件 |
| `initial_pose_x/y/yaw` | `0/0/0` | 初始位姿 |
| `initial_pose_delay_sec` | `4.0` | 初始位姿发布延迟 |
| `lifecycle_delay_sec` | `1.5` | 生命周期管理器启动延迟 |
| `use_sim_time` | `true` | 使用仿真时钟 |
| `spawn_x/y/z/yaw` | `0/0/0.03/0` | 机器人生成位姿 |

## 节点

| 节点 | 说明 |
|------|------|
| `nav2_lifecycle_runner` | 管理 Nav2 节点生命周期（激活 map_server, amcl, controller, planner 等） |
| `initial_pose_publisher` | 发布 AMCL 初始位姿 |
| `nav_goal_runner` | 发送 NavigateToPose 目标并检测机器人运动 |

## 坐标系

- 机器人根坐标系：`base_link`
- 里程计坐标系：`odom → base_link`
- 地图坐标系：`map → odom`
- Nav2 控制器速度输出经 `velocity_smoother` 平滑后发布到 `/cmd_vel`

## 地图

预置地图 `maps/Software_Museum.yaml` 和 `maps/Software_Museum.pgm`，对应 ISCAS Museum 仿真场景。

## 测试

```bash
cd src/navigation_sim_demo_ros2
python3 -m pytest test/ -v
```

5 项测试全部通过：模块导入、资产存在性检查、nav_goal_runner、nav2_lifecycle_runner、map_yaml 格式。

## 验证证据

- Nav2 栈生命周期激活成功
- `/cmd_vel` 接收到 Nav2 控制器输出
- `/odom` 显示机器人位置变化
- `nav_goal_runner` 检测到 `navigation-motion-detected`
