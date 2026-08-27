# action_demo

第 5 章实验包：动作通信练习。

- 包类型：`ament_python`
- ROS 2 Jazzy
- 依赖：`action_demo_interfaces`

## 安装

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/lab_code/ch05_lab src/action_demo_interfaces --ignore-src -r -y
```

## 简介

本包演示 ROS 2 Action 通信。服务端提供 `/do_dishes` 动作，使用异步执行回调，每秒反馈进度；客户端发送目标并接收结果。

## 节点 / 可执行说明

| 节点 | 角色 | Action | 说明 |
| --- | --- | --- | --- |
| `server` | 服务端 | `/do_dishes` | 异步 `execute` 回调，每秒发布 feedback |
| `client` | 客户端 | `/do_dishes` | 发送目标，等待结果 |

## 构建命令

```bash
cd <workspace>
colcon build --symlink-install --packages-select action_demo_interfaces action_demo
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

## 运行步骤

```bash
# 终端 1：启动服务端
ros2 run action_demo server

# 终端 2：启动客户端
ros2 run action_demo client
```

## 验证方法

```bash
ros2 interface show action_demo_interfaces/action/DoDishes
ros2 action list
colcon test --packages-select action_demo
colcon test-result --verbose
```

当前接口字段固定为 `dishwasher_id`、`total_dishes_cleaned` 和 `percent_complete`；测试会直接检查这三个字段。

## 运行结果截图

![action_demo 运行结果](../docs/images/result.png)
