# 第17章 实验代码：MoveIt2 基础与运动学规划

本章学习使用 MoveIt2 的 Python API 实现正运动学和逆运动学控制。

## 文件说明

| 文件 | 用途 | 运行方式 |
|------|------|----------|
| `fk_demo.py` | 正运动学（FK）演示。使用关节空间控制设置各关节的目标角度，控制机械臂运动 | `ros2 run moveit_fk_ik_lab fk_demo` |
| `ik_demo.py` | 逆运动学（IK）演示。设置末端执行器目标位姿（位置+姿态），由 MoveIt2 解算 IK 并运动 | `ros2 run moveit_fk_ik_lab ik_demo` |
| `fk_ik_exercise.py` | FK/IK 综合练习题。包含 TODO 填空，完成末端位姿设置、关节空间规划和命名目标控制 | `ros2 run moveit_fk_ik_lab fk_ik_exercise` |
| `rectangle_exercise.py` | 笛卡尔路径矩形轨迹规划。沿矩形路径的四个顶点做笛卡尔直线运动 | `ros2 run moveit_fk_ik_lab rectangle_exercise` |

## 运行说明

所有脚本均需先启动 xArm 机器人仿真环境：

```bash
# 终端1：启动纯 MoveIt + RViz 仿真
ros2 launch xarm_ros2_arm_only arm_only_move_group.launch.py

# 需要 Gazebo + ros2_control 时使用：
# ros2 launch xarm_ros2_arm_only arm_only.launch.py

# 终端2：运行实验脚本
cd src/lab_code/ch17_lab/

ros2 run moveit_fk_ik_lab fk_demo
```

### `fk_ik_exercise.py` TODO 练习

打开 `fk_ik_exercise.py`，完成以下 TODO：

1. 设置末端执行器目标位姿（位置 x=0.3, y=-0.3, z=0.3，姿态 rpy=0,0,-π/4）
2. 设置机械臂当前状态为初始状态
3. 规划并执行运动到目标位姿
4. 设置六关节角度目标值 `[-0.9, -1.0, 0.2, 0.9, -0.76, 1.5]` 并规划执行
5. 使用命名目标 `Home` 回到初始位置

### `rectangle_exercise.py` TODO 练习

补全第三个和第四个矩形顶点坐标，完成笛卡尔路径规划。
