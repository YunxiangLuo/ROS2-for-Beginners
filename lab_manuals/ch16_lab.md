# 第16章 实验：机械臂 URDF 建模与状态发布

## 当前仓库仿真验证：自定义 URDF 与课程 Wheeltec TF 对照

### 实验目标

先用本实验的三自由度 Xacro 在 RViz 中验证 link/joint 和关节状态，再用 `robot_sim_demo` 对照真实 Gazebo 机器人状态发布和传感器 TF。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 终端 1：按本实验参考代码启动自定义模型
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(xacro src/lab_code/ch16_lab/arm_urdf/urdf/three_dof_arm.xacro)"
```

```bash
# 终端 2：另行启动 Gazebo Wheeltec 对照 TF
ros2 launch robot_sim_demo gazebo2.launch.py \
  gui:=true rviz:=true drive:=false
ros2 run tf2_ros tf2_echo base_link laser_link
```

### 观察与验收

自定义模型的关节角变化应反映到 RViz RobotModel；Wheeeltec 侧可看到独立的传感器 TF。源码：`src/lab_code/ch16_lab/`、`src/robot_sim_demo/models/wheeltec_robot/model.sdf`。两套模型不能混用 frame 或关节名。

> **对应理论章节**：第25章《ROS2机械臂建模（URDF/Xacro）》
> **实验课时**：2课时  
> **实验代码**：`src/lab_code/ch16_lab/`（`arm_urdf/` 三自由度臂 URDF/Xacro + `arm_state_publisher/` 状态发布器）  

## 实验目标
- 掌握URDF语法和link/joint配置方法
- 学会创建3自由度机械臂的URDF模型
- 掌握Xacro的宏定义和数学计算功能
- 能够在RViz2中可视化自定义机械臂模型
- 学会用关节状态发布器驱动自建模型运动

## 实验环境
- ROS 2 Jazzy
- URDF/Xacro工具链
- RViz2可视化工具
- joint_state_publisher_gui

## 参考代码说明
`src/lab_code/ch16_lab/` 下的两个目录为**纯文件示例（非 colcon 功能包）**，无需构建，可直接阅读源码学习，也可将文件复制到自己的功能包中运行：

```
src/lab_code/ch16_lab/
├── arm_urdf/                      # 三自由度机械臂模型
│   ├── urdf/three_dof_arm.urdf    # URDF 版模型
│   ├── urdf/three_dof_arm.xacro   # Xacro 宏定义版模型
│   └── launch/display.launch.py   # RViz 显示 launch 文件
└── arm_state_publisher/
    └── state_publisher.py         # 向 /joint_states 发布关节角度
```

- `three_dof_arm.xacro` 定义了底座 + 3个旋转关节（`joint1` 绕Z轴，`joint2`/`joint3` 绕Y轴）与两个夹爪指关节（`finger1_joint`/`finger2_joint`），使用 `xacro:property` 常量（M_PI、阻尼、摩擦）与惯性矩阵宏（`cylinder_inertial`、`box_inertial`），与本实验16.5节练习的写法一致。
- `state_publisher.py` 以正弦/往复规律发布 `joint1`~`joint3` 与 `finger1_joint`、`finger2_joint` 的关节角（夹爪周期性开合），配合 `robot_state_publisher` 可在 RViz2 中观察模型运动，关节命名与 `three_dof_arm` 模型一致。

参考运行方式：
```bash
# 终端1: 加载Xacro并启动robot_state_publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(xacro src/lab_code/ch16_lab/arm_urdf/urdf/three_dof_arm.xacro)"

# 终端2: 运行状态发布器
python3 src/lab_code/ch16_lab/arm_state_publisher/state_publisher.py

# 终端3: 启动RViz2, Fixed Frame设为base_link, 添加RobotModel显示
rviz2
```

## 实验步骤

### 16.1 创建功能包
```bash
cd ~/ros2_arm_ws/src
ros2 pkg create my_arm_description --build-type ament_python --dependencies rclpy
cd my_arm_description
mkdir urdf launch rviz
```

### 16.2 创建3自由度机械臂URDF
创建 `urdf/simple_arm.urdf`（可对照参考实现 `src/lab_code/ch16_lab/arm_urdf/urdf/three_dof_arm.urdf`）:
```xml
<?xml version="1.0"?>
<robot name="simple_arm">
    <!-- 底座 -->
    <link name="base_link">
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
            <geometry><cylinder length="0.1" radius="0.08"/></geometry>
            <material name="gray">
                <color rgba="0.5 0.5 0.5 1"/>
            </material>
        </visual>
        <collision>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
            <geometry><cylinder length="0.1" radius="0.08"/></geometry>
        </collision>
        <inertial>
            <mass value="0.5"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
        </inertial>
    </link>

    <!-- 关节1: 绕Z轴旋转 (腰部) -->
    <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <origin rpy="0 0 0" xyz="0 0 0.1"/>
        <axis xyz="0 0 1"/>
        <limit effort="100" velocity="1.0" lower="-3.14" upper="3.14"/>
    </joint>

    <!-- 连杆1 -->
    <link name="link1">
        <visual>
            <geometry><box size="0.08 0.06 0.25"/></geometry>
            <origin rpy="0 0 0" xyz="0 0 0.125"/>
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>
        </visual>
        <collision>
            <geometry><box size="0.08 0.06 0.25"/></geometry>
            <origin rpy="0 0 0" xyz="0 0 0.125"/>
        </collision>
        <inertial>
            <mass value="0.3"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
        </inertial>
    </link>

    <!-- 关节2: 绕Y轴旋转 (肩部) -->
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
            <geometry><cylinder length="0.3" radius="0.025"/></geometry>
            <origin rpy="0 0 0" xyz="0 0 0.15"/>
            <material name="red">
                <color rgba="1 0 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry><cylinder length="0.3" radius="0.025"/></geometry>
            <origin rpy="0 0 0" xyz="0 0 0.15"/>
        </collision>
        <inertial>
            <mass value="0.2"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
        </inertial>
    </link>

    <!-- 关节3: 绕Y轴旋转 (肘部) -->
    <joint name="joint3" type="revolute">
        <parent link="link2"/>
        <child link="link3"/>
        <origin rpy="0 0 0" xyz="0 0 0.3"/>
        <axis xyz="0 1 0"/>
        <limit effort="100" velocity="1.0" lower="-2.09" upper="2.09"/>
    </joint>

    <!-- 连杆3 (末端) -->
    <link name="link3">
        <visual>
            <geometry><box size="0.03 0.03 0.15"/></geometry>
            <origin rpy="0 0 0" xyz="0 0 0.075"/>
            <material name="green">
                <color rgba="0 1 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry><box size="0.03 0.03 0.15"/></geometry>
            <origin rpy="0 0 0" xyz="0 0 0.075"/>
        </collision>
        <inertial>
            <mass value="0.1"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
        </inertial>
    </link>
</robot>
```

### 16.3 检查URDF语法
```bash
sudo apt install liburdfdom-tools -y
check_urdf urdf/simple_arm.urdf

# 生成模型结构图
urdf_to_graphiz urdf/simple_arm.urdf
# 查看生成的simple_arm.pdf
```

### 16.4 创建launch文件显示模型
创建 `launch/display_urdf.launch.py`:
```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('my_arm_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'simple_arm.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
        ),
    ])
```

在RViz2中查看模型:
```bash
cd ~/ros2_arm_ws
colcon build --packages-select my_arm_description
source install/setup.bash
ros2 launch my_arm_description display_urdf.launch.py
```

在RViz2中:
- 设置Fixed Frame为base_link
- 添加RobotModel显示插件
- 添加TF显示插件
- 拖动joint1/joint2/joint3滑块观察运动

### 16.5 创建Xacro版本模型
创建 `urdf/materials.xacro`:
```xml
<?xml version="1.0"?>
<robot name="materials" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <material name="gray"><color rgba="0.5 0.5 0.5 1"/></material>
    <material name="blue"><color rgba="0 0 1 1"/></material>
    <material name="red"><color rgba="1 0 0 1"/></material>
    <material name="green"><color rgba="0 1 0 1"/></material>
    <material name="orange"><color rgba="1 0.5 0 1"/></material>
</robot>
```

创建 `urdf/macros.xacro`:
```xml
<?xml version="1.0"?>
<robot name="macros" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:macro name="inertia_box" params="m w h d">
        <inertial>
            <mass value="${m}"/>
            <inertia ixx="${m*(h*h + d*d)/12}" ixy="0" ixz="0"
                     iyy="${m*(w*w + d*d)/12}" iyz="0"
                     izz="${m*(w*w + h*h)/12}"/>
        </inertial>
    </xacro:macro>

    <xacro:macro name="inertia_cylinder" params="m r l">
        <inertial>
            <mass value="${m}"/>
            <inertia ixx="${m*(3*r*r + l*l)/12}" ixy="0" ixz="0"
                     iyy="${m*(3*r*r + l*l)/12}" iyz="0"
                     izz="${m*r*r/2}"/>
        </inertial>
    </xacro:macro>
</robot>
```

创建 `urdf/simple_arm.xacro`（可对照参考实现 `src/lab_code/ch16_lab/arm_urdf/urdf/three_dof_arm.xacro`，后者在相同结构上增加了夹爪指关节）:
```xml
<?xml version="1.0"?>
<robot name="simple_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:property name="PI" value="3.1415926"/>
    <xacro:property name="damping" value="50.0"/>
    <xacro:property name="friction" value="1.0"/>

    <xacro:include filename="$(find my_arm_description)/urdf/materials.xacro"/>
    <xacro:include filename="$(find my_arm_description)/urdf/macros.xacro"/>

    <link name="base_link">
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
            <geometry><cylinder length="0.1" radius="0.08"/></geometry>
            <material name="gray"/>
        </visual>
        <collision>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
            <geometry><cylinder length="0.1" radius="0.08"/></geometry>
        </collision>
        <inertia_cylinder m="0.5" r="0.08" l="0.1"/>
    </link>

    <joint name="joint1" type="revolute">
        <parent link="base_link"/><child link="link1"/>
        <origin rpy="0 0 0" xyz="0 0 0.1"/>
        <axis xyz="0 0 1"/>
        <limit effort="100" velocity="1.0" lower="${-PI}" upper="${PI}"/>
        <dynamics damping="${damping}" friction="${friction}"/>
    </joint>

    <link name="link1">
        <visual>
            <geometry><box size="0.08 0.06 0.25"/></geometry>
            <origin rpy="0 0 0" xyz="0 0 0.125"/>
            <material name="blue"/>
        </visual>
        <collision>
            <geometry><box size="0.08 0.06 0.25"/></geometry>
            <origin rpy="0 0 0" xyz="0 0 0.125"/>
        </collision>
        <inertia_box m="0.3" w="0.08" h="0.06" d="0.25"/>
    </link>

    <joint name="joint2" type="revolute">
        <parent link="link1"/><child link="link2"/>
        <origin rpy="0 0 0" xyz="0 0 0.25"/>
        <axis xyz="0 1 0"/>
        <limit effort="100" velocity="1.0" lower="${-PI*2/3}" upper="${PI*2/3}"/>
        <dynamics damping="${damping}" friction="${friction}"/>
    </joint>

    <link name="link2">
        <visual>
            <geometry><cylinder length="0.3" radius="0.025"/></geometry>
            <origin rpy="0 0 0" xyz="0 0 0.15"/>
            <material name="red"/>
        </visual>
        <collision>
            <geometry><cylinder length="0.3" radius="0.025"/></geometry>
            <origin rpy="0 0 0" xyz="0 0 0.15"/>
        </collision>
        <inertia_cylinder m="0.2" r="0.025" l="0.3"/>
    </link>

    <joint name="joint3" type="revolute">
        <parent link="link2"/><child link="link3"/>
        <origin rpy="0 0 0" xyz="0 0 0.3"/>
        <axis xyz="0 1 0"/>
        <limit effort="100" velocity="1.0" lower="${-PI*2/3}" upper="${PI*2/3}"/>
        <dynamics damping="${damping}" friction="${friction}"/>
    </joint>

    <link name="link3">
        <visual>
            <geometry><box size="0.03 0.03 0.15"/></geometry>
            <origin rpy="0 0 0" xyz="0 0 0.075"/>
            <material name="green"/>
        </visual>
        <collision>
            <geometry><box size="0.03 0.03 0.15"/></geometry>
            <origin rpy="0 0 0" xyz="0 0 0.075"/>
        </collision>
        <inertia_box m="0.1" w="0.03" h="0.03" d="0.15"/>
    </link>
</robot>
```

### 16.6 测试Xacro转换
```bash
cd ~/ros2_arm_ws
source install/setup.bash

# 转换Xacro为URDF
xacro src/my_arm_description/urdf/simple_arm.xacro > /tmp/test_arm.urdf
check_urdf /tmp/test_arm.urdf

# 验证转换后的URDF
diff <(check_urdf /tmp/test_arm.urdf 2>&1) <(echo "")
```

### 16.7 添加夹爪
扩展模型，在link3末端添加夹爪。创建 `urdf/simple_arm_gripper.xacro`:
```xml
<?xml version="1.0"?>
<robot name="simple_arm_gripper" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:include filename="$(find my_arm_description)/urdf/simple_arm.xacro"/>

    <joint name="gripper_joint" type="fixed">
        <parent link="link3"/><child link="gripper_base"/>
        <origin rpy="0 0 0" xyz="0 0 0.15"/>
    </joint>

    <link name="gripper_base">
        <visual>
            <geometry><box size="0.06 0.04 0.02"/></geometry>
            <material name="orange"/>
        </visual>
    </link>

    <joint name="finger1_joint" type="prismatic">
        <parent link="gripper_base"/><child link="finger1"/>
        <origin rpy="0 0 0" xyz="0 -0.02 0"/>
        <axis xyz="0 1 0"/>
        <limit effort="10" velocity="0.5" lower="0" upper="0.03"/>
    </joint>

    <link name="finger1">
        <visual>
            <geometry><box size="0.02 0.04 0.05"/></geometry>
            <origin rpy="0 0 0" xyz="0 0.04 0.025"/>
            <material name="gray"/>
        </visual>
    </link>

    <joint name="finger2_joint" type="prismatic">
        <parent link="gripper_base"/><child link="finger2"/>
        <origin rpy="0 0 0" xyz="0 0.02 0"/>
        <axis xyz="0 -1 0"/>
        <limit effort="10" velocity="0.5" lower="0" upper="0.03"/>
    </joint>

    <link name="finger2">
        <visual>
            <geometry><box size="0.02 0.04 0.05"/></geometry>
            <origin rpy="0 0 0" xyz="0 0.04 0.025"/>
            <material name="gray"/>
        </visual>
    </link>
</robot>
```

创建 `launch/display_xacro.launch.py`:
```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('my_arm_description')
    xacro_path = os.path.join(pkg_path, 'urdf', 'simple_arm_gripper.xacro')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', xacro_path])}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
        ),
    ])
```

编译并查看完整模型:
```bash
cd ~/ros2_arm_ws
colcon build --packages-select my_arm_description
source install/setup.bash
ros2 launch my_arm_description display_xacro.launch.py
```

在RViz2中拖动所有滑块，包括finger1_joint和finger2_joint，观察夹爪开合。

也可以运行参考代码的状态发布器驱动该模型（关节命名已对齐 `three_dof_arm`）：
```bash
python3 src/lab_code/ch16_lab/arm_state_publisher/state_publisher.py
```

### 16.8 保存RViz2配置
在RViz2中完成显示配置后:
- 调整好视角 (按Ctrl+0重置视角)
- 按Ctrl+S保存配置到 `my_arm_description/rviz/display.rviz`
- 下次启动时可以在launch文件中指定该配置

### 16.9 模型参数修改练习
修改以下参数，观察效果变化:
```bash
# 修改连杆长度
# 在simple_arm.xacro中找到link1的box size，将z方向的0.25改为0.35

# 修改关节限位
# 将joint2的lower从-2.09改为-1.57，upper从2.09改为1.57

# 添加新的连杆
# 在link3后添加joint4和link4，增加一个自由度
```

## 实验结果与分析
- 通过URDF成功创建了3自由度机械臂模型，在RViz2中完整显示
- Xacro通过宏定义和数学表达式简化了模型描述，减少了重复代码
- check_urdf工具确保模型语法正确，urdf_to_graphiz生成了直观的模型结构图
- 夹爪使用prismatic关节实现了平移开合运动
- 参考代码 `arm_state_publisher/state_publisher.py` 驱动 `three_dof_arm` 模型实现了关节正弦运动与夹爪周期开合，验证了"模型+状态发布器"组合的正确性

## 思考题
1. URDF中revolute和prismatic关节有什么区别? 各适合什么应用场景?
2. Xacro的宏定义有哪些优势? 在复杂机器人建模中如何组织模块化Xacro文件?
3. inertial参数对仿真有什么影响? 如果全部设为0会怎样?
4. 如何为机械臂添加更多的自由度? 需要注意哪些约束?
