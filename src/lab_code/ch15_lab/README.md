# 第15章 实验代码：机械臂基础与关节空间控制

本章实验围绕 xArm 关节状态发布和夹爪开合控制展开。

## 文件说明

| 文件 | 用途 | 运行方式 |
|------|------|----------|
| `hello_arm_node.py` | ROS2 节点基础示例。创建 `JointState` 发布器，模拟关节运动并发布到 `/joint_states` 话题 | `ros2 run arm_joint_pub_lab hello_arm_node` |
| `arm_joints_pub1.py` | 关节状态发布器。发布 8 个关节的 `JointState` 消息，让 `arm_2_joint` 在 ±1.5 rad 范围内往复摆动 | `ros2 run arm_joint_pub_lab arm_joints_pub1` |
| `arm_gripper.py` | 机械臂+手爪联动演示。关节 1、4 和手爪关节循环往复运动 | `ros2 run arm_joint_pub_lab arm_gripper` |
| `gripper_open_close.py` | 手爪张开/闭合演示。`gripper_1_joint` 和 `gripper_2_joint` 同步开合 | `ros2 run arm_joint_pub_lab gripper_open_close` |
| `checker3` | 串口设备检测工具。检测激光雷达、机械臂、PCB 板等 USB 串口设备，自动写入 udev 端口映射规则 | 详见 `README.md` |

## 运行说明

启动 xArm 的 MoveIt + RViz 仿真（在另一个终端）：

```bash

ros2 launch xarm_ros2_arm_only arm_only_move_group.launch.py
```

运行关节发布脚本：

```bash
cd src/lab_code/ch15_lab/
ros2 run arm_joint_pub_lab arm_joints_pub1

# 在 RViz2 中添加 /joint_states 话题查看关节运动
# 或使用命令行查看：
ros2 topic echo /joint_states
```

## checker3 使用说明

`checker3` 是 Linux 下的串口检测脚本，用于自动检测和配置连接到开发板的 USB 串口设备。详见 `checker3` 目录下的 README。
