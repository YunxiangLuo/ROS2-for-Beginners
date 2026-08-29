# name_demo_cpp

ROS 2 Jazzy（ament_cmake）C++ 命名空间与参数示例，演示节点全名、命名空间以及参数的设置与读取。

## 可执行程序

| 可执行 | 说明 |
| --- | --- |
| `name_demo_node` | 节点名 `name_demo`。打印节点全名、命名空间以及参数 `serial`、`global_serial` |

### 参数

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `serial` | int | -1 | 命名空间内序号，launch 中设置为 10 |
| `global_serial` | int | -1 | 全局序号，launch 中设置为 5 |

## 依赖

- `rclcpp`
- `launch`（exec_depend）
- `launch_ros`（exec_depend）

## 构建

```bash

cd robot_sim_demo

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select name_demo_cpp

source install/setup.bash
```

## 运行

通过 launch 文件启动（在 `academy` 命名空间下，设置 `serial=10`、`global_serial=5`）：

```bash
ros2 launch name_demo_cpp demo.launch.py
```

直接运行节点（默认命名空间与参数）：

```bash

ros2 run name_demo_cpp name_demo_node
```

## Launch 文件

| 文件 | 说明 |
| --- | --- |
| `demo.launch.py` | 在 `academy` 命名空间下启动 `name_demo_node`，并设置 `serial=10`、`global_serial=5` |
