# 第18章 实验代码：MoveIt2 规划场景、避障与抓取放置

本章学习规划场景（Planning Scene）的构建、障碍物添加、物体附着以及 3D 目标发布。

## 文件说明

| 文件 | 用途 | 运行方式 |
|------|------|----------|
| `beeline_demo.py` | 笛卡尔直线路径规划。沿三角形路径的三个顶点做笛卡尔直线运动 | `ros2 run moveit_pick_place_lab beeline_demo` |
| `obstacles_demo.py` | 障碍物避障演示。在规划场景中添加桌子、球体和长方体，观察机械臂避障运动 | `ros2 run moveit_pick_place_lab obstacles_demo` |
| `target_publisher.py` | 3D 目标点发布器。在 RViz 中可视化一个动态移动的目标球体，发布 `PoseStamped` 到 `/target_pose` | `ros2 run moveit_pick_place_lab target_publisher` |
| `attach_object_demo.py` | 物体附着与抓取演示。在规划场景中添加障碍物，将一个工具附着到末端执行器，执行抓取放置 | `ros2 run moveit_pick_place_lab attach_object_demo` |
| `pick_place_demo.py` | Pick & Place 综合演示 | `ros2 run moveit_pick_place_lab pick_place_demo` |

## 运行说明

```bash
# 终端1：启动纯 MoveIt + RViz 仿真（仅适合观察规划轨迹）
ros2 launch xarm_ros2_arm_only arm_only_move_group.launch.py

# 执行型 demo（beeline_demo / obstacles_demo / attach_object_demo /
# pick_place_demo）需要控制器在线，请改用：
# ros2 launch xarm_ros2_arm_only arm_only.launch.py

# 终端2：运行脚本
cd src/lab_code/ch18_lab/

ros2 run moveit_pick_place_lab beeline_demo

# 对于带用户交互的脚本，按 Enter 键逐步执行
ros2 run moveit_pick_place_lab obstacles_demo
```

### `target_publisher.py`

该节点发布一个正弦运动的 3D 目标点，可通过 RViz 添加 `Marker` 和 `PoseStamped` 显示查看。支持参数调整：

```bash
ros2 run moveit_pick_place_lab target_publisher --ros-args \
  -p rate:=20 \
  -p speed:=1.5 \
  -p target_frame:=base_link
```
