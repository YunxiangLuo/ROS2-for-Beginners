# 第25章 ROS2机械臂建模

## 学习目标
- 掌握URDF和Xacro建模方法
- 理解SRDF语义描述文件的结构
- 学会在Rviz中可视化机械臂模型
- 掌握3D模型导入与整合方法

## 25.1 URDF建模基础

### 25.1.1 URDF概述

URDF（Unified Robot Description Format，统一机器人描述格式）是ROS2中标准的机器人模型描述格式。它基于XML规范，用于描述机器人的运动学、动力学和几何信息。

URDF的核心结构将机器人建模为由连杆（Link）和关节（Joint）组成的树状结构：

- **连杆（Link）**：描述刚体的外观和物理属性
- **关节（Joint）**：描述连杆之间的连接和相对运动关系

URDF文件的基本结构：

```xml
<?xml version="1.0"?>
<robot name="my_arm">
    <!-- 定义底座 -->
    <link name="base_link">
        <visual>
            <geometry>
                <cylinder length="0.18" radius="0.1"/>
            </geometry>
            <material name="yellow">
                <color rgba="1 1 0 1"/>
            </material>
        </visual>
    </link>

    <!-- 定义关节 -->
    <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <origin xyz="0 0 0.18" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit effort="300" velocity="0.6" lower="-2.96" upper="2.96"/>
    </joint>

    <link name="link1">
        <visual>
            <geometry>
                <box size="0.1 0.06 0.2"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>
        </visual>
    </link>
</robot>
```

### 25.1.2 link元素详解

`<link>`元素描述连杆的外观和物理属性，包含三个核心子元素：

**visual（视觉属性）**：

```xml
<link name="arm_link1">
    <visual>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <geometry>
            <box size="0.1 0.06 0.2"/>
        </geometry>
        <material name="blue">
            <color rgba="0 0 1 1"/>
        </material>
    </visual>
</link>
```

**collision（碰撞属性）**：用于碰撞检测的简化模型，可与visual形状不同：

```xml
<link name="arm_link1">
    <collision>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <geometry>
            <box size="0.1 0.06 0.2"/>
        </geometry>
    </collision>
</link>
```

**inertial（惯性属性）**：用于动力学仿真：

```xml
<link name="arm_link1">
    <inertial>
        <origin xyz="0 0 0.09" rpy="0 0 0"/>
        <mass value="1.0"/>
        <inertia ixx="0.0052" ixy="0" ixz="0"
                 iyy="0.0052" iyz="0" izz="0.005"/>
    </inertial>
</link>
```

### 25.1.3 joint元素详解

`<joint>`元素描述连杆之间的连接关系：

**关节类型**：

| 类型 | 说明 | 自由度 | 示例 |
|------|------|--------|------|
| revolute | 旋转关节（有限位） | 1 | 机械臂关节 |
| continuous | 连续旋转关节 | 1 | 车轮 |
| prismatic | 滑动关节 | 1 | 夹爪 |
| fixed | 固定连接 | 0 | 底座 |
| floating | 浮动连接 | 6 | 自由物体 |
| planar | 平面运动 | 3 | AGV底盘 |

**关节属性**：

```xml
<joint name="joint1" type="revolute">
    <parent link="base_link"/>            <!-- 父连杆 -->
    <child link="link1"/>                 <!-- 子连杆 -->
    <origin xyz="0 0 0.18" rpy="0 0 0"/> <!-- 相对位姿 -->
    <axis xyz="0 0 1"/>                   <!-- 关节轴线方向 -->
    <limit effort="300" velocity="0.6"    <!-- 运动限制 -->
           lower="-2.96" upper="2.96"/>
    <dynamics damping="50" friction="1"/> <!-- 动力学属性 -->
    <mimic joint="joint2" multiplier="1" offset="0"/> <!-- 模仿关节 -->
    <safety_controller soft_lower_limit="-2.85"        <!-- 安全限制 -->
                       soft_upper_limit="2.85"
                       k_position="100"/>
</joint>
```

## 25.2 六自由度机械臂URDF建模

### 25.2.1 创建功能包

```bash
# 创建工作空间
mkdir -p ~/ros2_arm_ws/src
cd ~/ros2_arm_ws/src

# 创建描述功能包
ros2 pkg create arm_description --build-type ament_python --dependencies rclpy

# 创建目录结构
cd arm_description
mkdir -p urdf launch rviz meshes
```

### 25.2.2 完整六自由度机械臂URDF

```xml
<?xml version="1.0"?>
<robot name="six_dof_arm">
    <!-- 底座 -->
    <link name="base_link">
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0.045"/>
            <geometry>
                <cylinder length="0.09" radius="0.08"/>
            </geometry>
            <material name="gray">
                <color rgba="0.5 0.5 0.5 1"/>
            </material>
        </visual>
        <collision>
            <origin rpy="0 0 0" xyz="0 0 0.045"/>
            <geometry>
                <cylinder length="0.09" radius="0.08"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="2.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0"
                     iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
    </link>

    <!-- 关节1：腰部旋转 -->
    <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <origin rpy="0 0 0" xyz="0 0 0.09"/>
        <axis xyz="0 0 1"/>
        <limit effort="100" velocity="1.0" lower="-3.14" upper="3.14"/>
    </joint>

    <!-- 连杆1 -->
    <link name="link1">
        <visual>
            <geometry>
                <box size="0.06 0.06 0.25"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.125"/>
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.06 0.06 0.25"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.125"/>
        </collision>
        <inertial>
            <mass value="1.5"/>
            <inertia ixx="0.005" ixy="0" ixz="0"
                     iyy="0.005" iyz="0" izz="0.001"/>
        </inertial>
    </link>

    <!-- 关节2：肩部旋转 -->
    <joint name="joint2" type="revolute">
        <parent link="link1"/>
        <child link="link2"/>
        <origin rpy="0 0 0" xyz="0 0 0.25"/>
        <axis xyz="0 1 0"/>
        <limit effort="100" velocity="1.0" lower="-2.09" upper="2.09"/>
    </joint>

    <!-- 连杆2 -->
    <link name="link2">
        <visual>
            <geometry>
                <box size="0.06 0.06 0.25"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.125"/>
            <material name="green">
                <color rgba="0 1 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.06 0.06 0.25"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.125"/>
        </collision>
        <inertial>
            <mass value="1.2"/>
            <inertia ixx="0.004" ixy="0" ixz="0"
                     iyy="0.004" iyz="0" izz="0.001"/>
        </inertial>
    </link>

    <!-- 关节3：肘部旋转 -->
    <joint name="joint3" type="revolute">
        <parent link="link2"/>
        <child link="link3"/>
        <origin rpy="0 0 0" xyz="0 0 0.25"/>
        <axis xyz="0 1 0"/>
        <limit effort="100" velocity="1.0" lower="-2.09" upper="2.09"/>
    </joint>

    <!-- 连杆3 -->
    <link name="link3">
        <visual>
            <geometry>
                <box size="0.05 0.05 0.15"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.075"/>
            <material name="red">
                <color rgba="1 0 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.05 0.05 0.15"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.075"/>
        </collision>
        <inertial>
            <mass value="0.8"/>
            <inertia ixx="0.002" ixy="0" ixz="0"
                     iyy="0.002" iyz="0" izz="0.001"/>
        </inertial>
    </link>

    <!-- 关节4：腕部旋转1 -->
    <joint name="joint4" type="continuous">
        <parent link="link3"/>
        <child link="link4"/>
        <origin rpy="0 0 0" xyz="0 0 0.15"/>
        <axis xyz="0 0 1"/>
        <limit effort="50" velocity="1.0"/>
    </joint>

    <!-- 连杆4 -->
    <link name="link4">
        <visual>
            <geometry>
                <box size="0.04 0.04 0.08"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.04"/>
            <material name="yellow">
                <color rgba="1 1 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.04 0.04 0.08"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.04"/>
        </collision>
        <inertial>
            <mass value="0.3"/>
            <inertia ixx="0.001" ixy="0" ixz="0"
                     iyy="0.001" iyz="0" izz="0.0005"/>
        </inertial>
    </link>

    <!-- 关节5：腕部旋转2 -->
    <joint name="joint5" type="continuous">
        <parent link="link4"/>
        <child link="link5"/>
        <origin rpy="0 0 0" xyz="0 0 0.08"/>
        <axis xyz="0 1 0"/>
        <limit effort="50" velocity="1.0"/>
    </joint>

    <!-- 连杆5（末端） -->
    <link name="link5">
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0.04"/>
            <geometry>
                <cylinder length="0.08" radius="0.03"/>
            </geometry>
            <material name="gray">
                <color rgba="0.7 0.7 0.7 1"/>
            </material>
        </visual>
        <collision>
            <origin rpy="0 0 0" xyz="0 0 0.04"/>
            <geometry>
                <cylinder length="0.08" radius="0.03"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="0.2"/>
            <inertia ixx="0.0005" ixy="0" ixz="0"
                     iyy="0.0005" iyz="0" izz="0.0001"/>
        </inertial>
    </link>

    <!-- 工具坐标系 -->
    <link name="tool0"/>
    <joint name="joint6" type="fixed">
        <parent link="link5"/>
        <child link="tool0"/>
        <origin rpy="0 0 0" xyz="0 0 0.08"/>
    </joint>
</robot>
```

## 25.3 Xacro建模

### 25.3.1 Xacro简介

Xacro（XML Macros）是URDF的宏语言扩展，提供了变量定义、数学运算、条件判断和宏模板等高级功能，使模型描述更加简洁和可维护。

### 25.3.2 Xacro基本语法

**命名空间和变量**：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro"
       name="six_dof_arm">

    <!-- 定义可配置参数 -->
    <xacro:property name="base_radius" value="0.08"/>
    <xacro:property name="base_height" value="0.09"/>
    <xacro:property name="link1_length" value="0.25"/>
    <xacro:property name="link2_length" value="0.25"/>
    <xacro:property name="link3_length" value="0.15"/>

    <!-- 数学运算 -->
    <xacro:property name="base_origin_z" value="${base_height / 2}"/>
    <xacro:property name="total_height" value="${base_height + link1_length + link2_length + link3_length}"/>
```

**宏定义（Macro）**：

```xml
<!-- 定义连杆宏 -->
<xacro:macro name="link_visual" params="name length color">
    <link name="${name}">
        <visual>
            <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
            <geometry>
                <box size="0.06 0.06 ${length}"/>
            </geometry>
            <material name="${color}">
                <color rgba="${color}"/>
            </material>
        </visual>
        <collision>
            <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
            <geometry>
                <box size="0.06 0.06 ${length}"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="${length * 5}"/>
            <inertia ixx="${length * 0.01}" ixy="0" ixz="0"
                     iyy="${length * 0.01}" iyz="0" izz="${length * 0.005}"/>
        </inertial>
    </link>
</xacro:macro>

<!-- 使用宏 -->
<xacro:link_visual name="link1" length="${link1_length}" color="1 0 0 1"/>
<xacro:link_visual name="link2" length="${link2_length}" color="0 1 0 1"/>
```

**条件判断**：

```xml
<xacro:if value="${use_gripper}">
    <!-- 包含夹爪 -->
    <xacro:include filename="gripper.xacro"/>
</xacro:if>
```

**循环生成**：

```xml
<xacro:macro name="joint_limit" params="joint_name lower upper">
    <limit effort="100" velocity="1.0"
           lower="${lower}" upper="${upper}"/>
</xacro:macro>

<!-- 在关节中使用 -->
<joint name="joint2" type="revolute">
    <xacro:joint_limit joint_name="joint2"
                       lower="${-pi*2/3}" upper="${pi*2/3}"/>
</joint>
```

### 25.3.3 使用Xacro重构URDF

将前面的URDF模型用Xacro重构，可大幅减少重复代码：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro"
       name="six_dof_arm">

    <xacro:property name="pi" value="3.14159265359"/>

    <!-- 可配置参数 -->
    <xacro:property name="base_radius" value="0.08"/>
    <xacro:property name="link_width" value="0.06"/>
    <xacro:property name="link1_len" value="0.25"/>
    <xacro:property name="link2_len" value="0.25"/>
    <xacro:property name="link3_len" value="0.15"/>

    <!-- 颜色定义 -->
    <material name="blue">
        <color rgba="0 0 1 1"/>
    </material>
    <material name="green">
        <color rgba="0 1 0 1"/>
    </material>
    <material name="red">
        <color rgba="1 0 0 1"/>
    </material>

    <!-- 连杆宏 -->
    <xacro:macro name="create_link" params="name length color_name">
        <link name="${name}">
            <visual>
                <origin xyz="0 0 ${length/2}"/>
                <geometry>
                    <box size="${link_width} ${link_width} ${length}"/>
                </geometry>
                <material name="${color_name}"/>
            </visual>
            <collision>
                <origin xyz="0 0 ${length/2}"/>
                <geometry>
                    <box size="${link_width} ${link_width} ${length}"/>
                </geometry>
            </collision>
            <inertial>
                <mass value="${length * 5}"/>
                <inertia ixx="${0.001 * length * 10}" ixy="0" ixz="0"
                         iyy="${0.001 * length * 10}" iyz="0" izz="${0.001}"/>
            </inertial>
        </link>
    </xacro:macro>

    <!-- 旋转关节宏 -->
    <xacro:macro name="create_revolute_joint"
                 params="name parent child axis origin_xyz lower upper">
        <joint name="${name}" type="revolute">
            <parent link="${parent}"/>
            <child link="${child}"/>
            <origin xyz="${origin_xyz}" rpy="0 0 0"/>
            <axis xyz="${axis}"/>
            <limit effort="100" velocity="1.0"
                   lower="${lower}" upper="${upper}"/>
        </joint>
    </xacro:macro>

    <!-- 底座 -->
    <link name="base_link">
        <visual>
            <origin xyz="0 0 ${base_radius/2}"/>
            <geometry>
                <cylinder length="${base_radius + 0.01}"
                          radius="${base_radius}"/>
            </geometry>
            <material name="gray">
                <color rgba="0.5 0.5 0.5 1"/>
            </material>
        </visual>
    </link>

    <!-- 关节1 -->
    <xacro:create_revolute_joint
        name="joint1" parent="base_link" child="link1"
        axis="0 0 1" origin_xyz="0 0 ${base_radius}"
        lower="${-pi}" upper="${pi}"/>

    <!-- 连杆1 -->
    <xacro:create_link name="link1" length="${link1_len}" color_name="blue"/>

    <!-- 关节2 -->
    <xacro:create_revolute_joint
        name="joint2" parent="link1" child="link2"
        axis="0 1 0" origin_xyz="0 0 ${link1_len}"
        lower="${-pi*2/3}" upper="${pi*2/3}"/>

    <!-- 连杆2 -->
    <xacro:create_link name="link2" length="${link2_len}" color_name="green"/>

    <!-- 关节3 -->
    <xacro:create_revolute_joint
        name="joint3" parent="link2" child="link3"
        axis="0 1 0" origin_xyz="0 0 ${link2_len}"
        lower="${-pi*2/3}" upper="${pi*2/3}"/>

    <!-- 连杆3 -->
    <xacro:create_link name="link3" length="${link3_len}" color_name="red"/>
</robot>
```

## 25.4 SRDF语义描述

### 25.4.1 SRDF的作用

SRDF（Semantic Robot Description Format）是对URDF的语义补充，为MoveIt2等高级框架提供额外的语义信息，包括：

- 规划组（Planning Groups）：定义关节集合
- 预设位姿（Named Poses）：预定义的关节位置
- 自碰撞对（Collision Pairs）：不需要碰撞检测的连杆对
- 末端执行器（End Effectors）：定义工具信息
- 虚拟关节（Virtual Joints）：连接机器人与世界坐标系

### 25.4.2 SRDF文件示例

```xml
<?xml version="1.0"?>
<robot name="six_dof_arm">
    <!-- 规划组：定义手臂的所有关节 -->
    <group name="arm_group">
        <joint name="joint1"/>
        <joint name="joint2"/>
        <joint name="joint3"/>
        <joint name="joint4"/>
        <joint name="joint5"/>
        <joint name="joint6"/>
    </group>

    <!-- 规划组：定义夹爪的所有关节 -->
    <group name="gripper">
        <joint name="gripper_left_joint"/>
        <joint name="gripper_right_joint"/>
    </group>

    <!-- 预设位姿 -->
    <group_state name="home" group="arm_group">
        <joint name="joint1" value="0"/>
        <joint name="joint2" value="0"/>
        <joint name="joint3" value="0"/>
        <joint name="joint4" value="0"/>
        <joint name="joint5" value="0"/>
        <joint name="joint6" value="0"/>
    </group_state>

    <group_state name="vertical" group="arm_group">
        <joint name="joint1" value="0"/>
        <joint name="joint2" value="-1.57"/>
        <joint name="joint3" value="1.57"/>
        <joint name="joint4" value="0"/>
        <joint name="joint5" value="1.57"/>
        <joint name="joint6" value="0"/>
    </group_state>

    <!-- 末端执行器 -->
    <end_effector name="gripper_ee"
                  parent_link="link5"
                  parent_group="arm_group"
                  group="gripper"/>

    <!-- 虚拟关节（将机器人连接到世界坐标系） -->
    <virtual_joint name="base_footprint"
                   type="fixed"
                   parent_frame="world"
                   child_link="base_link"/>

    <!-- 自碰撞免检对 -->
    <disable_collisions link1="base_link" link2="link2" reason="Never"/>
    <disable_collisions link1="base_link" link2="link3" reason="Never"/>
    <disable_collisions link1="base_link" link2="link4" reason="Never"/>
    <disable_collisions link1="base_link" link2="link5" reason="Never"/>
    <disable_collisions link1="link1" link2="link4" reason="Adjacent"/>
    <disable_collisions link1="link1" link2="link5" reason="Never"/>
</robot>
```

### 25.4.3 MoveIt2的SRDF使用

MoveIt2通过Setup Assistant工具自动生成SRDF文件。用户也可以在配置包中手动创建或修改SRDF：

```yaml
# MoveIt2配置包的目录结构
generated_moveit_config/
├── config/
│   ├── six_dof_arm.srdf         # SRDF语义描述
│   ├── kinematics.yaml           # 运动学配置
│   ├── joint_limits.yaml         # 关节极限
│   ├── ompl_planning.yaml        # OMPL规划器配置
│   ├── fake_controllers.yaml     # 虚拟控制器
│   └── ros2_controllers.yaml     # 实际控制器
└── launch/
    ├── move_group.launch.py
    ├── planning_context.launch.py
    └── demo.launch.py
```

## 25.5 在Rviz中可视化

### 25.5.1 创建启动文件

```python
# launch/display.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('arm_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'six_dof_arm.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')]
        ),
    ])
```

### 25.5.2 Xacro版本的启动文件

```python
# launch/display_xacro.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('arm_description')

    # Xacro处理参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_file = os.path.join(pkg_share, 'urdf', 'six_dof_arm.xacro')

    # 将Xacro转换为URDF
    from xacro import process_file
    doc = process_file(xacro_file)
    robot_description = doc.toxml()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
        ),
    ])
```

### 25.5.3 启动模型可视化

```bash
# 查看URDF模型
ros2 launch arm_description display.launch.py

# 查看Xacro模型
ros2 launch arm_description display_xacro.launch.py
```

### 25.5.4 命令行工具检查模型

```bash
# 检查URDF语法
check_urdf six_dof_arm.urdf

# 使用URDF工具包
ros2 run urdfdom_py urdf_to_graphviz six_dof_arm.urdf

# 查看模型信息
ros2 topic echo /robot_description --once

# 查看TF树
ros2 run tf2_tools view_frames.py
```

## 25.6 3D模型导入

### 25.6.1 使用Mesh文件

URDF支持导入3D网格文件（STL、DAE格式）作为连杆的视觉和碰撞模型：

```xml
<link name="base_link">
    <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <mesh filename="package://arm_description/meshes/base.stl"
                  scale="1.0 1.0 1.0"/>
        </geometry>
        <material name="gray">
            <color rgba="0.5 0.5 0.5 1"/>
        </material>
    </visual>
    <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <mesh filename="package://arm_description/meshes/base_collision.stl"
                  scale="1.0 1.0 1.0"/>
        </geometry>
    </collision>
</link>
```

### 25.6.2 3D模型文件格式

| 格式 | 优点 | 缺点 |
|------|------|------|
| STL（.stl） | 广泛支持、二进制格式小 | 只有几何、无颜色 |
| Collada（.dae） | 支持颜色纹理材质 | 文件较大 |
| OBJ（.obj） | 通用格式 | 需配套MTL文件 |

### 25.6.3 创建简化碰撞模型

为提升碰撞检测性能，通常为复杂的可视化模型创建简化的碰撞模型：

```python
# 使用Trimesh库简化网格
import trimesh
import numpy as np

def simplify_mesh(input_path, output_path, max_faces=500):
    """简化3D网格用于碰撞检测"""
    mesh = trimesh.load(input_path)
    simplified = mesh.simplify_quadric_decimation(max_faces)
    simplified.export(output_path)
    print(f"简化: {len(mesh.faces)} → {len(simplified.faces)} 面")

# 示例：简化底座模型
simplify_mesh(
    'meshes/base.stl',
    'meshes/base_collision.stl',
    max_faces=500
)
```

### 25.6.4 检查网格文件路径

```bash
# 检查包路径
echo $AMENT_PREFIX_PATH

# 列出包中的网格文件
find ~/ros2_arm_ws/install/arm_description -name "*.stl" -o -name "*.dae"
```

## 25.7 模型验证与调试

### 25.7.1 使用check_urdf工具

```bash
# 安装检查工具
sudo apt install liburdfdom-tools

# 检查URDF模型
check_urdf six_dof_arm.urdf

# 输出示例：
# robot name is: six_dof_arm
# ---------- Successfully Parsed XML ----------
# root link: base_link has 1 child(ren)
#   child(1): link1
#     child(1): link2
#       ...
```

### 25.7.2 生成模型关系图

```bash
# 安装graphviz
sudo apt install graphviz

# 生成URDF图形表示
urdf_to_graphviz six_dof_arm.urdf

# 输出文件: six_dof_arm.pdf
```

### 25.7.3 Rviz调试技巧

在Rviz中添加以下显示组件进行调试：

1. **RobotModel**：显示机器人模型
2. **TF**：显示坐标系
3. **MotionPlanning**：MoveIt2规划显示

## 课后练习

1. 编写一个六自由度机械臂的URDF文件，包含底座、三个连杆、腕部和末端执行器，使用不同的颜色区分各部件。

2. 使用Xacro重构上述URDF，定义连杆长度、颜色等参数为变量，并使用宏简化代码。

3. 创建一个SRDF文件，定义arm_group和gripper两个规划组，并添加home和vertical两个预设位姿。

4. 编写launch文件，在Rviz中可视化机械臂模型，并使用joint_state_publisher_gui手动控制各关节运动。

5. 导入一个3D模型（STL或DAE）作为机械臂连杆的可视化模型，并创建简化的碰撞检测模型。

---

## 仿真结合实例（当前仓库）：xArm Xacro、SRDF 与 RViz RobotModel

### 目标与知识点对应

把本章的 URDF/Xacro、SRDF 规划组和 RViz RobotModel 放入 xArm6 仿真，观察机器人描述如何同时被 `robot_state_publisher`、MoveIt2 和 Gazebo 使用。

### 运行步骤

需要先提供外部兼容的 `xarm_description` 2.0.0：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source /path/to/xarm_description_workspace/install/setup.bash
xacro src/xarm/urdf/arm_only_xarm.urdf.xacro \
  hardware_type:=mock_components use_ros2_control:=false \
  include_world_joint:=true base_height:=0.0 > /tmp/arm_only.urdf
xmllint --noout /tmp/arm_only.urdf
ros2 launch xarm_ros2_arm_only arm_only_move_group.launch.py use_rviz:=true
```

### 观察结果

RViz 中应显示 xArm 连杆、关节和 `xarm` 规划组；改变 Xacro 参数后重新展开，可比较底座高度等几何参数的影响。

### 源码与边界

- Xacro：`src/xarm/urdf/arm_only_xarm.urdf.xacro`
- SRDF：`src/xarm/config/xarm.srdf`
- RViz：`src/xarm/config/arm_only_moveit.rviz`

底层描述包和网格不随本仓库提供；本实例验证描述文件和配置关系，不替代真实机械臂标定。
