# 第8章 PPT：URDF/Xacro 机器人建模

> 共 15 页，标注页码 · 图号与教学文档对应

---

## P1 · 标题页

**URDF/Xacro 机器人建模**

- 课程：ROS2 Python 编程
- 章节：第 8 章
- 课时：2 课时

---

## P2 · 本课学习目标

- 理解 URDF 文件结构：`<link>` 与 `<joint>` 两大基本元素
- 掌握 link 的 visual / collision / inertial 三类子元素
- 掌握 joint 类型与 `limit` 限位要求
- 理解 `origin` 的坐标语义差异
- 用 XACRO 属性、数学表达式、宏实现参数化建模
- 用 robot_state_publisher + RViz2 RobotModel 可视化验证

---

## P3 · URDF 文件结构

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

---

## P4 · link 元素详解

| 子元素 | 用途 | 示例 |
|--------|------|------|
| `<visual>` | 可视化几何体 (mesh/box/cylinder/sphere) | `<cylinder radius="0.1" length="0.2"/>` |
| `<collision>` | 碰撞检测几何体（通常与 visual 一致） | `<box size="0.4 0.3 0.15"/>` |
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

---

## P5 · joint 类型

| 类型 | 运动自由度 | 典型应用 |
|------|-----------|---------|
| `revolute` | 绕轴旋转（有限范围） | 机械臂关节 |
| `continuous` | 无限旋转 | 轮子 |
| `prismatic` | 沿轴平移 | 升降机构 |
| `fixed` | 无自由度 | 传感器固定连接 |
| `planar` | 平面运动 | 地面移动 |

- revolute / continuous / prismatic 均需声明 `limit`（上下界、最大力矩、最大速度），这是 ros2_control 与 MoveIt 2 正常工作的前提
- 模型描述还包含 `floating` 类型与 `gazebo` 扩展标签

---

## P6 · origin 语义与建模流程

- 初学者最易踩坑：`origin` 语义随父元素而变
  - link 的 `origin`：几何体「相对自身惯性中心」的位姿
  - joint 的 `origin`：「子 link 相对父 link」的变换
- 官方推荐的开发循环：编辑 URDF → `check_urdf` 语法检查 → RViz 中 `RobotModel` + `TF` 双显示验证
- 网格模型（mesh）用于复杂外观，支持 `scale` 缩放
- 物理属性优先于外观：惯性张量不可全为零（仿真报 NaN），碰撞几何应比视觉几何更简化

---

## P7 · XACRO 基础宏

```xml
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

---

## P8 · mesh 文件引用

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

- Mesh 文件通常放在 `meshes/` 目录，路径使用 `package://<pkg_name>/` 前缀
- `xacro` 展开命令：`ros2 run xacro xacro 模型.xacro > 输出.urdf`

---

## P9 · XACRO 三大武器与模块化

- 属性（`<xacro:property>`）：常量集中定义，如 `wheel_radius`
- 数学表达式（`${(wheel_radius * 2)/3}`）：内联计算，消除魔法数字
- 宏（`<xacro:macro>`）：一次定义、参数化复用，系列化产品改属性即可生成多套 URDF
- `<xacro:if>` / `<xacro:unless>`：条件分支，如 `use_lidar=true` 时才加载 lidar_link
- `<xacro:include>`：大型模型按「底盘 / 机械臂 / 传感器」分文件维护
- 传感器套件（相机、激光雷达、IMU）作为可选模块按需拼装

---

## P10 · robot_state_publisher + joint_state_publisher

```python
# launch/display.launch.py
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

## P11 · RViz2 RobotModel 显示

- 添加 `RobotModel` 插件，Description Topic 设置为 `/robot_description`
- 添加 `TF` 插件查看各坐标系，Fixed Frame 设为 `base_link` 或 `odom`
- 将配置保存为 `.rviz` 文件，供 Launch 用 `-d` 参数加载
- 同一份模型文件，可被 RViz 用于可视化、Gazebo 用于物理仿真、MoveIt 2 用于运动规划
- 调试顺序建议：先 RViz 静态验证、再 Gazebo 动力学验证、最后接入 ros2_control

---

## P12 · 仿真结合实例：从 Xacro 模型到 RViz RobotModel

```bash
# 终端 1：展开并校验 Xacro
ros2 run xacro xacro \
  src/urdf_demo_ros2/urdf/mybot.xacro > /tmp/mybot.urdf
xmllint --noout /tmp/mybot.urdf

# 终端 2：启动 robot_state_publisher + RViz
ros2 launch urdf_demo_ros2 display_xacro.launch.py \
  use_gui:=false use_rviz:=true
```

- RViz 中 RobotModel 显示 Xacro 展开的连杆与关节，TF 面板可看到坐标树
- `xmllint` 通过表示 Xacro 输出是合法 XML
- 该入口与 Gazebo SDF 模型路径不同，用于对比 URDF/Xacro 与 SDF 两种建模方式

---

## P13 · 本章要点

1. URDF 由 link（刚体）与 joint（连接）构成，link 含 visual / collision / inertial 三类描述
2. joint 分为 revolute、continuous、prismatic、fixed 等类型，运动型需 `limit`
3. `origin` 语义随父元素而变，是初学者最常踩的坑
4. XACRO 通过宏、属性与数学表达式消除重复并支持条件分支
5. `robot_state_publisher` 解析 URDF 并发布 TF，`joint_state_publisher` 提供关节值 GUI 滑块
6. RViz2 的 RobotModel 插件可视化验证完整机器人模型

---

## P14 · 练习题

1. 编写包含 base_link 和 lidar_link 的 URDF，通过 fixed joint 连接
2. 为 base_link 添加 collision 和 inertial 元素（mass=10kg, box inertia）
3. 使用 XACRO macro 参数化生成 4 个麦克纳姆轮（前左/前右/后左/后右）
4. 编写 `display.launch.py`，启动 robot_state_publisher + joint_state_publisher + rviz2
5. 在 RViz2 中调出 RobotModel 和 TF 显示，截图完整的机器人模型
6. 使用 XACRO `<xacro:if>` 条件判断：当 `use_lidar=true` 时才加载 lidar_link

---

## P15 · 下章预告

**第 9 章：Gazebo 仿真**

- Gazebo 版本选择与 URDF → SDF 桥接
- spawn_entity 机器人生成
- LiDAR / Camera / IMU 传感器插件
- ros2_control 差速驱动控制