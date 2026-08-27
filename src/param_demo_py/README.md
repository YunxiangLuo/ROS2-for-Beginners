# param_demo_py

ROS 2 Jazzy（ament_python）Python 参数系统示例，演示参数的声明、读取、设置、更新与删除。

## 节点

| 节点（可执行） | 说明 |
| --- | --- |
| `param_demo` | 节点名 `param_demo_py`。声明 6 个参数，每秒循环打印参数列表：第 1 次将 `param2` 更新为 2，第 2 次通过 `undeclare_parameter` 删除 `param5`，达到 `max_loops=3` 后自动关闭 |

### 参数

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `param1` | int | 111 | 示例参数 |
| `param2` | int | 222 | 第 1 次循环更新为 2 |
| `param3` | int | 33333 | 示例参数 |
| `param4` | int | 0 | 初始化后设置为 4 |
| `param5` | int | 0 | 初始化后设置为 5，第 2 次循环删除 |
| `max_loops` | int | 3 | 达到该循环次数后关闭节点 |

## 依赖

- `rclpy`

## 构建

```bash
cd robot_sim_demo
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select param_demo_py
source install/setup.bash
```

## 运行

```bash
ros2 run param_demo_py param_demo
```

查询参数：

```bash
ros2 param list /param_demo_py
ros2 param get /param_demo_py param1
```

## 测试

```bash
colcon test --packages-select param_demo_py
```

1 项测试通过（模块导入冒烟测试）。
