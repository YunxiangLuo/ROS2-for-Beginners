# tf_follower_ros2

ROS 2 Jazzy（ament_python）TF 跟随控制器，从 ROS 1 的 `tf_follower` 迁移而来。节点监听 TF 变换，根据目标相对位姿计算并发布 `Twist` 速度命令，驱动机器人跟随目标。

## 节点

| 节点（可执行） | 说明 |
| --- | --- |
| `tf_follower` | 监听 `follower_frame -> target_frame` 变换，计算线速度与角速度并发布到 `cmd_vel` 话题 |
| `fake_target_broadcaster` | 广播模拟目标位置的 TF 变换，支持 `static`（静止）与 `circle`（圆周运动）两种模式 |

## 依赖

- `rclpy`
- `geometry_msgs`
- `tf2_ros_py`
- `launch`（exec_depend）
- `launch_ros`（exec_depend）

## 构建

```bash

cd robot_sim_demo

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select tf_follower_ros2

source install/setup.bash
```

## 运行

启动完整演示（`fake_target_broadcaster` 圆周运动 + `tf_follower`，速度发布到 `/mybot_cmd_vel`）：

```bash
ros2 launch tf_follower_ros2 tf_follower_demo.launch.py
```

仅启动 `tf_follower`（需外部提供目标 TF）：

```bash

ros2 launch tf_follower_ros2 tf_follower.launch.py
```

## Launch 文件

| 文件 | 说明 |
| --- | --- |
| `tf_follower_demo.launch.py` | 启动 `fake_target_broadcaster`（`circle` 模式）与 `tf_follower`，速度发布到 `/mybot_cmd_vel` |
| `tf_follower.launch.py` | 仅启动 `tf_follower` |

## 参数

### `tf_follower`

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `follower_frame` | string | `mybot_link` | 跟随者（机器人）坐标系 |
| `target_frame` | string | `base_footprint` | 目标坐标系 |
| `cmd_vel_topic` | string | `/mybot_cmd_vel` | 速度命令发布话题 |
| `stop_distance` | double | 1.0 | 停止距离（小于该距离时停止） |
| `linear_gain` | double | 0.1 | 线速度增益 |
| `angular_gain` | double | -0.4 | 角速度增益 |
| `max_linear_speed` | double | 1.0 | 最大线速度 |
| `max_angular_speed` | double | 1.5 | 最大角速度 |
| `lookup_rate_hz` | double | 10.0 | TF 查询频率（Hz） |

### `fake_target_broadcaster`

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `parent_frame` | string | `mybot_link` | 父坐标系 |
| `child_frame` | string | `base_footprint` | 子坐标系（目标） |
| `motion_mode` | string | `static` | 运动模式：`static` 或 `circle` |
| `x` | double | 3.0 | 静止模式下的 x 坐标 |
| `y` | double | 1.0 | 静止模式下的 y 坐标 |
| `z` | double | 0.0 | z 坐标 |
| `center_x` | double | 3.0 | 圆周运动圆心 x 坐标 |
| `center_y` | double | 0.0 | 圆周运动圆心 y 坐标 |
| `radius` | double | 1.0 | 圆周运动半径 |
| `angular_speed` | double | 0.5 | 圆周运动角速度（rad/s） |
| `period_sec` | double | 0.1 | 广播周期（秒） |

## 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/mybot_cmd_vel` | `geometry_msgs/msg/Twist` | `tf_follower` 发布的速度命令 |

## 测试

```bash
colcon test --packages-select tf_follower_ros2
```

7 项测试全部通过：控制器单元测试（3 项）、目标位置计算测试（2 项）、集成测试（1 项）、模块导入冒烟测试（1 项）。
