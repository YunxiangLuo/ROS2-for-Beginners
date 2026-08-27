# 第8章 PPT：URDF/Xacro 机器人建模

> 共 15 页

---

## P1 · 标题页
**URDF/Xacro 机器人建模** | 第8章 | 2课时

## P2 · 学习目标
- 掌握 URDF link 和 joint 语法
- 理解 visual / collision / inertial 三维
- 使用 XACRO 宏和参数化建模
- robot_state_publisher + joint_state_publisher
- RViz2 RobotModel 可视化

## P3 · URDF 基本结构

```
<robot name="robot_name">
  <link name="link1"/>        刚体描述
  <link name="link2"/>
  <joint name="j1" type="...">  连接关系
    <parent link="link1"/>
    <child link="link2"/>
  </joint>
</robot>
```

## P4 · link 三大子元素

| 元素 | 用途 | 必需 |
|------|------|------|
| `<visual>` | 可视化 (mesh/形状/颜色) | 推荐 |
| `<collision>` | 碰撞检测 | 推荐 |
| `<inertial>` | 质量和惯量矩阵 | 仿真必需 |

## P5 · joint 类型表

| 类型 | 自由度 | 用途 |
|------|-------|------|
| `fixed` | 0 | 传感器固定 |
| `revolute` | 1 旋转 (有限) | 机械臂 |
| `continuous` | 1 旋转 (无限) | 轮子 |
| `prismatic` | 1 平移 | 升降 |
| `planar` | 3 (x,y,yaw) | 地面 |

## P6 · URDF 完整示例

```xml
<link name="base_link">
  <visual>
    <geometry><box size="0.4 0.3 0.15"/></geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  <collision>...</collision>
  <inertial><mass value="5.0"/></inertial>
</link>
```

## P7 · XACRO 属性变量

程序 8-2：
```xml
<xacro:property name="wheel_radius" value="0.05"/>
<cylinder radius="${wheel_radius}" length="0.03"/>
```

一键修改参数，全局生效

## P8 · XACRO 宏定义

```xml
<xacro:macro name="wheel" params="prefix reflect">
  <joint name="${prefix}_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="${prefix}_wheel"/>
  </joint>
</xacro:macro>

<xacro:wheel prefix="left" reflect="1"/>
<xacro:wheel prefix="right" reflect="-1"/>
```

## P9 · XACRO 条件分支

```xml
<xacro:if value="${sensor_type == 'lidar'}">
  <link name="lidar_link">...</link>
</xacro:if>
<xacro:unless value="${disabled}">
  ...
</xacro:unless>
```

## P10 · 机器人状态发布架构

```
URDF/Xacro 文件
      ↓ xacro 解析
  /robot_description (topic)
      ↓
robot_state_publisher ──→ /tf (所有 link 位姿)
      ↑
joint_state_publisher (GUI slider / 传感器数据)
```

## P11 · display.launch.py

程序 8-3：
```python
Node(package='robot_state_publisher', ...)
Node(package='joint_state_publisher', ...)
Node(package='rviz2', ...)
```

三步启动：robot_state_publisher + joint_state_publisher + RViz2

## P12 · RViz2 可视化

1. 添加 RobotModel → 选 Description Topic
2. 添加 TF → 显示坐标系轴
3. Fixed Frame: `base_link`
4. joint_state_publisher 拖动滑块测试关节

## P13 · Mesh 文件使用

```xml
<mesh filename="package://my_pkg/meshes/lidar.stl"/>
```

支持 STL / DAE (Collada) 格式

## P14 · 本章要点

1. URDF = link (含 visual/collision/inertial) + joint
2. joint 类型：fixed / revolute / continuous / prismatic
3. XACRO 宏消除重复，属性实现参数化
4. robot_state_publisher 发布 TF，joint_state_publisher 控制关节
5. RViz2 RobotModel + TF 插件可视化

## P15 · 下章预告
**第9章：Gazebo 仿真** — 机器人物理仿真，传感器插件，ros2_control 运动控制