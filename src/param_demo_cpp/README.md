# param_demo_cpp

ROS 2 Jazzy（ament_cmake）C++ 参数系统示例，演示参数的声明、读取、设置与循环更新。

## 可执行程序

| 可执行 | 说明 |
| --- | --- |
| `param_demo_cpp_node` | 节点名 `param_demo_cpp`。声明 6 个参数，每秒循环打印参数列表：第 1 次将 `param2` 更新为 2，达到 `max_loops=3` 后自动关闭 |

### 参数

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `param1` | int | 111 | 示例参数 |
| `param2` | int | 222 | 第 1 次循环更新为 2 |
| `param3` | int | 33333 | 示例参数 |
| `param4` | int | 0 | 初始化后设置为 4 |
| `param5` | int | 0 | 初始化后设置为 5 |
| `max_loops` | int | 3 | 达到该循环次数后关闭节点 |

## 依赖

- `rclcpp`

## 构建

```bash

cd robot_sim_demo

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select param_demo_cpp

source install/setup.bash
```

## 运行

```bash
ros2 run param_demo_cpp param_demo_cpp_node
```

查询参数：

```bash

ros2 param list /param_demo_cpp

ros2 param get /param_demo_cpp param1
```
