# tf_demo_py

ROS 2 Jazzy（ament_python）Python TF2 坐标变换示例，演示如何广播与查询坐标变换，以及坐标变换数学工具的使用。

## 节点

| 节点（可执行） | 说明 |
| --- | --- |
| `py_tf_broadcaster` | 每秒广播 `base_link -> link1` 变换，yaw 从 1.57 起始每秒递增 0.1，平移为 (1, 2, 3) |
| `py_tf_listener` | 每秒查询 `base_link -> link1` 变换并打印平移与四元数 |
| `coordinate_transformation` | 坐标变换数学工具：向量点乘、范数、归一化，欧拉角/四元数互转，旋转矩阵计算 |

## 依赖

- `rclpy`
- `geometry_msgs`
- `tf2_ros_py`

## 构建

```bash

cd robot_sim_demo

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select tf_demo_py

source install/setup.bash
```

## 运行

终端 1 启动广播器：

```bash
ros2 run tf_demo_py py_tf_broadcaster
```

终端 2 启动监听器：

```bash

ros2 run tf_demo_py py_tf_listener
```

也可使用官方工具查看变换：

```bash
ros2 run tf2_ros tf2_echo base_link link1
```

运行坐标变换数学工具：

```bash

ros2 run tf_demo_py coordinate_transformation
```

## 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/tf` | `tf2_msgs/msg/TFMessage` | `py_tf_broadcaster` 发布的 `base_link -> link1` 变换 |

## 测试

```bash
colcon test --packages-select tf_demo_py
```

1 项测试通过（模块导入冒烟测试）。
