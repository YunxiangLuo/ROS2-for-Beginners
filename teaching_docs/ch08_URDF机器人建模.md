# 第8章：URDF/Xacro 机器人建模

## 仿真结合实例（当前仓库）：从 Xacro 模型到 RViz RobotModel

### 目标与知识点对应

先用 `xacro` 展开课程提供的模型，再由 `robot_state_publisher` 发布 TF，并在 RViz 中显示 RobotModel，能够把本章的 link/joint、Xacro 参数化和状态发布串成一条可运行链路。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 终端 1：展开并校验 Xacro
ros2 run xacro xacro \
  src/urdf_demo_ros2/urdf/mybot.xacro > /tmp/mybot.urdf
xmllint --noout /tmp/mybot.urdf

# 终端 2：启动 robot_state_publisher + RViz
ros2 launch urdf_demo_ros2 display_xacro.launch.py \
  use_gui:=false use_rviz:=true
```

需要对照 Gazebo 模型时，再开一个终端：

```bash
source install/setup.bash
ros2 launch robot_sim_demo gazebo2.launch.py gui:=true rviz:=false drive:=false
```

### 观察结果

- RViz 的 RobotModel 能显示 Xacro 展开的连杆和关节，TF 面板能看到模型的坐标树。
- `xmllint` 通过表示 Xacro 输出是合法 XML；Gazebo 侧则使用其 SDF Wheeltec 模型和传感器配置。

### 源码与边界

- Xacro：`src/urdf_demo_ros2/urdf/mybot.xacro`
- Launch：`src/urdf_demo_ros2/launch/display_xacro.launch.py`
- Gazebo 模型：`src/robot_sim_demo/models/wheeltec_robot/model.sdf`

两个入口用于对比 URDF/Xacro 与 SDF 的建模路径；不要把 Wheeltec 的 SDF 文件当作 `urdf_demo_ros2` 的 Xacro 源文件。

> **课程**：ROS2 Python 编程  
> **章节**：第8章  
> **课时**：2 课时（90 分钟）  

---

## 8.1 URDF 语法基础

### 知识点 8.1.1：URDF 概述与文件结构

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- 连杆定义 -->
  <link name="base_link">
    <visual> ... </visual>
    <collision> ... </collision>
    <inertial> ... </inertial>
  </link>

  <!-- 关节定义 -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

程序 8-1：URDF 文件最小结构。每个 `<link>` 描述刚体，每个 `<joint>` 描述连接关系。

### 知识点 8.1.2：link 元素详解

| 子元素 | 用途 | 示例 |
|--------|------|------|
| `<visual>` | 可视化几何体 (mesh/box/cylinder/sphere) | `<cylinder radius="0.1" length="0.2"/>` |
| `<collision>` | 碰撞检测几何体 (通常与 visual 一致) | `<box size="0.4 0.3 0.15"/>` |
| `<inertial>` | 惯性参数 (mass + inertia) | `<mass value="1.0"/>` |

```xml
<link name="base_link">
  <visual>
    <geometry><box size="0.4 0.3 0.15"/></geometry>
    <material name="blue"><color rgba="0 0 0.8 1"/></material>
  </visual>
  <collision>
    <geometry><box size="0.4 0.3 0.15"/></geometry>
  </collision>
  <inertial>
    <mass value="5.0"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
  </inertial>
</link>
```

### 知识点 8.1.3：joint 类型

| 类型 | 运动自由度 | 典型应用 |
|------|-----------|---------|
| `revolute` | 绕轴旋转 (有限范围) | 机械臂关节 |
| `continuous` | 无限旋转 | 轮子 |
| `prismatic` | 沿轴平移 | 升降机构 |
| `fixed` | 无自由度 | 传感器固定连接 |
| `planar` | 平面运动 | 地面移动 |

---

## 8.2 XACRO 宏与参数化

### 知识点 8.2.1：XACRO 基础宏

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xbot">

  <!-- 参数化属性 -->
  <xacro:property name="wheel_radius" value="0.05"/>
  <xacro:property name="wheel_width" value="0.03"/>

  <!-- 宏定义 -->
  <xacro:macro name="wheel" params="name prefix reflect">
    <joint name="${name}_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="0 ${reflect*wheel_base/2} -0.1" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <!-- 宏实例化 -->
  <xacro:wheel name="left" prefix="left" reflect="1"/>
  <xacro:wheel name="right" prefix="right" reflect="-1"/>
</robot>
```

程序 8-2：XACRO 宏减少重复代码，参数化实现车轮批量生成。

### 知识点 8.2.2：mesh 文件引用

```xml
<link name="lidar_link">
  <visual>
    <geometry>
      <mesh filename="package://robot_description/meshes/lidar.stl"/>
    </geometry>
    <origin xyz="0 0 0.02" rpy="0 0 0"/>
  </visual>
</link>
```

Mesh 文件通常放在 `meshes/` 目录，使用 `package://<pkg_name>/` 路径前缀。

---

## 8.3 robot_state_publisher + joint_state_publisher

### 知识点 8.3.1：TF 自动发布机器人状态

```python
# launch/display.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('urdf_demo'),
        'urdf', 'simple_robot.xacro')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('urdf_demo'),
                'rviz', 'display.rviz')],
        ),
    ])
```

程序 8-3：`robot_state_publisher` 解析 URDF 并发布 TF，`joint_state_publisher` 提供关节状态 GUI。

---

## 8.4 RViz2 RobotModel 显示

### 知识点 8.4.1：RViz2 配置

1. 添加 `RobotModel` 插件，设置 `Description Topic: /robot_description`
2. 添加 `TF` 插件查看坐标系
3. 设置 Fixed Frame 为 `base_link` 或 `odom`
4. 保存配置为 `.rviz` 文件供 Launch 加载

---

## 8.5 本章小结

1. URDF = link (刚体) + joint (连接)，包含 visual/collision/inertial
2. joint 类型：revolute / continuous / prismatic / fixed
3. XACRO = 宏 + 参数变量，消除重复，支持条件分支
4. `robot_state_publisher` 解析 URDF → 发布 TF
5. `joint_state_publisher` 提供关节值 GUI 滑块
6. RViz2 RobotModel 插件可视化完整机器人模型

---

## 8.6 练习题

**练习 8.1**：编写一个包含 base_link 和 lidar_link 的 URDF，通过 fixed joint 连接。

**练习 8.2**：为 base_link 添加 collision 和 inertial 元素（mass=10kg, box inertia）。

**练习 8.3**：使用 XACRO macro 参数化生成 4 个麦克纳姆轮（前左/前右/后左/后右）。

**练习 8.4**：编写 `display.launch.py`，启动 robot_state_publisher + joint_state_publisher + rviz2。

**练习 8.5**：在 RViz2 中调出 RobotModel 和 TF 显示，截图完整的机器人模型。

**练习 8.6**：使用 XACRO `<xacro:if>` 条件判断：当 `use_lidar=true` 时才加载 lidar_link。
