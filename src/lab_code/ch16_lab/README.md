# 第16章 实验代码：机械臂 URDF 建模与状态发布

本目录是纯文件示例，不包含 `package.xml`、`CMakeLists.txt` 或可直接启动的 ROS 2 包。

## 目录结构

```text

src/lab_code/ch16_lab/

├── arm_urdf/

│   ├── launch/display.launch.py

│   └── urdf/

│       ├── three_dof_arm.urdf

│       └── three_dof_arm.xacro

└── arm_state_publisher/

    └── state_publisher.py
```

## 文件说明

| 文件/目录 | 用途 |
|-----------|------|
| `arm_urdf/urdf/three_dof_arm.urdf` | 三自由度机械臂的静态 URDF 示例 |
| `arm_urdf/urdf/three_dof_arm.xacro` | 参数化 Xacro 模型 |
| `arm_state_publisher/state_publisher.py` | 发布关节状态以驱动模型运动的 Python 示例 |

## 运行说明

从课程根目录加载 Jazzy 环境并展开 Xacro：

```bash
source /opt/ros/jazzy/setup.bash
xacro src/lab_code/ch16_lab/arm_urdf/urdf/three_dof_arm.xacro > /tmp/three_dof_arm.urdf
```

在一个终端启动 `robot_state_publisher`，在另一终端启动 `joint_state_publisher`，然后用 RViz2 查看 TF 与模型：

```bash

ros2 run robot_state_publisher robot_state_publisher \

  --ros-args -p robot_description:="$(xacro src/lab_code/ch16_lab/arm_urdf/urdf/three_dof_arm.xacro)"

ros2 run joint_state_publisher joint_state_publisher \

  --ros-args -p robot_description:="$(xacro src/lab_code/ch16_lab/arm_urdf/urdf/three_dof_arm.xacro)"

rviz2
```

若要使用课程提供的状态发布示例，请停止 `joint_state_publisher` 后运行：

```bash
python3 src/lab_code/ch16_lab/arm_state_publisher/state_publisher.py
```
