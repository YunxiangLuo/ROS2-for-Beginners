# slam_sim_demo_ros2

SLAM 仿真演示包：使用 slam_toolbox 在 ISCAS Museum 仿真场景中实现在线建图。

## 依赖

- `robot_sim_demo`（Gazebo 仿真环境）
- `slam_toolbox`
- `nav2_map_server`

```bash

sudo apt install -y ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server
```

## 构建

```bash
cd robot_sim_demo
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select slam_sim_demo_ros2
source install/setup.bash
```

## 运行

### 在线建图

```bash
# 终端 1：启动 Gazebo 仿真（不自动巡航）
ros2 launch robot_sim_demo gazebo2.launch.py gui:=false rviz:=false drive:=false

# 终端 2：启动 slam_toolbox 在线建图
ros2 launch slam_sim_demo_ros2 slam_demo.launch.py \

  use_gazebo:=false use_rviz:=false use_sim_time:=true

# 终端 3：驱动机器人建图
ros2 run slam_sim_demo_ros2 slam_map_runner --ros-args -p use_sim_time:=true
```

### 深度相机建图

```bash
ros2 launch slam_sim_demo_ros2 slam_depth_demo.launch.py \
  use_gazebo:=false use_rviz:=false use_sim_time:=true
```

### 一键启动

```bash

ros2 launch slam_sim_demo_ros2 slam_demo.launch.py \

  use_gazebo:=true use_rviz:=false
```

## Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_gazebo` | `false` | 是否同时启动 Gazebo |
| `gz_headless` | `true` | Gazebo 无头模式 |
| `use_rviz` | `true` | 启动 RViz2 |
| `slam_params_file` | `slam_toolbox_params.yaml` | slam_toolbox 参数 |
| `scan_topic` | `/scan` | 激光雷达话题 |
| `use_sim_time` | `true` | 使用仿真时钟 |
| `spawn_x/y/z/yaw` | `0/0/0.03/0` | 机器人生成位姿 |

## 节点

| 节点 | 说明 |
|------|------|
| `slam_map_runner` | 自动驱动机器人建图，监控地图增长、扫描更新和里程计距离 |
| `slam_save_reload_runner` | 保存/重载 SLAM 地图 |

## slam_map_runner 运动策略

机器人按以下循环运动（每 20 秒一个周期）：
- 0-8 秒：直行 `(0.18, 0.0)` m/s
- 8-12 秒：原地转弯 `(0.0, 0.5)` rad/s
- 12-20 秒：直行 `(0.18, 0.0)` m/s

成功条件：地图更新 ≥2 次、扫描更新 ≥2 次、移动距离 >0.15m、已知栅格增长 >20。

## 测试

```bash
cd src/slam_sim_demo_ros2
python3 -m pytest test/ -v
```

6 项测试全部通过：模块导入、资产存在性、已知栅格计数、平面距离计算、运动命令序列、save_reload runner。

## 验证证据

- slam_toolbox 注册传感器：`Custom Described Lidar`
- 7 次地图更新，已知栅格增长 22
- 移动距离 0.898m，42 帧扫描
- 输出 `slam-map-updated`
