# 第 5 章：动作通信

本章使用 `action_demo` 和工作区已有的 `action_demo_interfaces` 演示 ROS 2 Action。目标是洗碗机编号 `dishwasher_id`，服务端按 20%、40%、60%、80%、100% 发布进度，结果为 `total_dishes_cleaned`。

## 安装

需要 ROS 2 Jazzy、Python pytest，并确保工作区已有 `action_demo_interfaces`：

```bash

source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src/lab_code/ch05_lab src/action_demo_interfaces --ignore-src -r -y
```

## 构建

```bash
colcon build --symlink-install --packages-select \
  action_demo_interfaces action_demo
source install/setup.bash
```

## 运行

终端 1 启动服务端：

```bash

ros2 run action_demo server
```

终端 2 启动客户端：

```bash
ros2 run action_demo client
```

也可以直接查看接口：

```bash

ros2 interface show action_demo_interfaces/action/DoDishes
```

## 验证

```bash
ros2 action list
ros2 action info /do_dishes
colcon test --packages-select action_demo
colcon test-result --verbose
```

测试使用 `step_duration_sec:=0` 的伪目标句柄，检查真实接口字段和五次进度反馈，不等待 5 秒动作时长。

## 运行结果

实际运行结果截图：

![Ch05 动作通信运行结果](docs/images/result.png)
