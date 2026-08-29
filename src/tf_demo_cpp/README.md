# tf_demo_cpp

ROS 2 Jazzy（ament_cmake）C++ TF2 坐标变换示例，演示欧拉角/四元数转换、坐标变换运算以及 TF 广播与监听。

## 可执行程序

| 可执行 | 说明 |
| --- | --- |
| `euler_to_quaternion` | 从标准输入读取 `roll pitch yaw`，输出对应四元数 (w, x, y, z) |
| `quaternion_to_euler` | 从标准输入读取 `w x y z`，输出对应欧拉角 (roll, pitch, yaw) |
| `coordinate_transformation` | 坐标变换运算：点乘、长度、归一化、夹角、距离、叉乘、四元数与旋转矩阵 |
| `tf_broadcaster` | 每秒广播 `base_link -> link1` 变换，yaw 从 1.57 起始每秒递增 0.1，平移为 (1, 2, 3) |
| `tf_listener` | 每秒查询 `base_link -> link1` 变换并打印平移与四元数 |

## 依赖

- `rclcpp`
- `geometry_msgs`
- `tf2`
- `tf2_geometry_msgs`
- `tf2_ros`

## 构建

```bash

cd robot_sim_demo

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select tf_demo_cpp

source install/setup.bash
```

## 运行

广播与监听 TF：

```bash
# 终端 1
ros2 run tf_demo_cpp tf_broadcaster
# 终端 2
ros2 run tf_demo_cpp tf_listener
```

欧拉角转四元数（交互式输入）：

```bash

ros2 run tf_demo_cpp euler_to_quaternion
# 输入示例：0 0 1.57
```

四元数转欧拉角（交互式输入）：

```bash
ros2 run tf_demo_cpp quaternion_to_euler
# 输入示例：1 0 0 0
```

坐标变换运算：

```bash

ros2 run tf_demo_cpp coordinate_transformation
```

## 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/tf` | `tf2_msgs/msg/TFMessage` | `tf_broadcaster` 发布的 `base_link -> link1` 变换 |
