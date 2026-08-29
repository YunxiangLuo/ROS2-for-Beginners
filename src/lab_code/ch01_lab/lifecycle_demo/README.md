# lifecycle_demo

第 1 章正式 ROS 2 Python 实验包，演示生命周期节点从 `unconfigured` 到 `inactive`、再到 `active` 的状态转换。

## 安装

```bash

source /opt/ros/jazzy/setup.bash
```

## 构建

```bash
colcon build --symlink-install --packages-select lifecycle_demo
source install/setup.bash
```

## 运行

```bash

ros2 run lifecycle_demo hello_node

ros2 launch lifecycle_demo lifecycle_demo.launch.py autostart:=true
```

默认运行需要手动执行 `configure` 和 `activate`；launch 的 `autostart:=true` 会自动完成这两个转换。

## 验证

```bash
ros2 lifecycle get /hello_ros2_lifecycle
ros2 topic echo /cmd_vel --once
colcon test --packages-select lifecycle_demo
```

预期节点为 `active`，并收到 `linear.x=0.1` 的 `Twist` 消息。测试只使用 rclpy 节点，不需要图形界面。

## 运行结果截图

![lifecycle_demo 运行结果](../docs/images/result.png)
