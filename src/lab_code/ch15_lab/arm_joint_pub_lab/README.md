# arm_joint_pub_lab — 第 16 章机械臂关节发布实验

- 包类型：`ament_python`
- ROS 2 Jazzy + xArm6（`xarm_ros2_arm_only`）

## 简介

本章练习通过 `/joint_states` 话题驱动机械臂模型运动。`arm_joints_pub1`、

`arm_gripper`、`gripper_open_close` 使用 xArm 关节命名

（`arm_1_joint` 至 `arm_6_joint`、`gripper_1_joint`、`gripper_2_joint`），

配合 `robot_state_publisher` 可直接可视化 xArm URDF；`hello_arm_node` 为

通用教学示例（`joint1`/`finger` 命名，适用于任意简单模型）。

## 构建

```bash

cd <robot_sim_demo 工作区>

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select arm_joint_pub_lab

source install/setup.bash
```

## 运行

```bash
# 发布关节状态（xArm 关节命名）
ros2 run arm_joint_pub_lab arm_joints_pub1
ros2 run arm_joint_pub_lab arm_gripper
ros2 run arm_joint_pub_lab gripper_open_close

# 通用示例
ros2 run arm_joint_pub_lab hello_arm_node
```

验证（另开终端）：

```bash

ros2 topic echo /joint_states --once
```

## 测试

```bash
colcon test --packages-select arm_joint_pub_lab
colcon test-result --all
```

## 运行结果

`/joint_states` 中关节位置持续变化；配合 `robot_state_publisher` +

RViz 可见机械臂往复运动。截图保存至 `docs/images/arm_joint_pub.png`。
