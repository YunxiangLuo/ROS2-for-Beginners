# 第27章 MoveIt2 Python规划

> **课程**：ROS2 Python 编程  
> **章节**：第27章  
> **课时**：2 课时（90 分钟）  
> **教学方式**：讲授 + 演示  

---

## 学习目标

本章学习目标包括：掌握MoveItPy API的核心类和用法，学会使用PlanningComponent进行运动规划，理解关节空间运动规划的方法，掌握正逆运动学求解的编程实现。

## 27.1 MoveItPy API概览

### 27.1.1 MoveItPy简介

MoveItPy是MoveIt2的Python API封装，提供了控制机械臂运动规划的全套接口。通过MoveItPy，开发者可以用Python编程方式实现关节空间运动规划、笛卡尔空间路径规划、规划场景与碰撞检测、物体抓取与放置和运动学求解等功能。

### 27.1.2 核心类

**MoveItPy** — 主入口类，负责加载配置、初始化规划场景监听器：

```python
from moveit.planning import MoveItPy

# 初始化MoveItPy
moveit = MoveItPy(node_name='moveit_py')
```

**PlanningComponent** — 对应MoveGroup概念，封装了规划与执行接口：

```python
from moveit.planning import PlanningComponent

# 获取规划组件
arm = PlanningComponent(
    moveit,          # MoveItPy实例
    'arm_group',     # 规划组名称
    'link5'          # 末端执行器link
)
```

**RobotState** — 表示机器人状态，包含各关节位置：

```python
from moveit.core.robot_state import RobotState

robot_model = moveit.get_robot_model()
robot_state = RobotState(robot_model)
robot_state.set_joint_group_positions(
    'arm_group',
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
)
```

### 27.1.3 编程基本流程

使用MoveItPy API编程通常遵循以下步骤：

```python
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningComponent

class MoveIt2Programmer(Node):
    def __init__(self):
        super().__init__('moveit2_programmer')

        self.moveit = MoveItPy(node_name='moveit_py')
        self.arm = PlanningComponent(
            self.moveit, 'arm_group', 'link5'
        )

    def plan_and_execute(self):
        # 步骤1：设置起始状态
        self.arm.set_start_state_to_current_state()

        # 步骤2：设置目标（关节空间或笛卡尔空间）
        self.arm.set_joint_value_target([0.5, -0.3, 0.0, 0.0, 0.0, 0.0])

        # 步骤3：规划轨迹
        plan_result = self.arm.plan()

        # 步骤4：执行轨迹
        if plan_result:
            self.arm.execute(plan_result.trajectory)
            self.get_logger().info('轨迹执行完成')
        else:
            self.get_logger().error('规划失败')

def main(args=None):
    rclpy.init(args=args)
    node = MoveIt2Programmer()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 27.1.4 配置参数

MoveItPy的配置可通过参数文件或直接传入参数：

```python
from moveit.planning import MoveItPy

# 通过ROS2参数配置
moveit = MoveItPy(
    node_name='moveit_py',
    config_files=[
        'config/kinematics.yaml',
        'config/ompl_planning.yaml',
    ]
)
```

### 27.1.5 官方要点——MoveItPy 官方教程的六个组件编程模型

> 本节内容综合翻译自 MoveIt 2 官方文档（moveit.picknik.ai 的 MoveItPy Tutorial Suite）、PickNik Academy 的 MoveItPy 课程与 MoveIt 官方示例仓库（ros-planning/moveit2 的 moveit_py 包），另参考 The Construct 的 MoveIt Python 课程与 Robotics Back-End 的 MoveItPy 实战教程。原文均为英文，此处为中文编译，供课后巩固与进阶阅读。

MoveIt 官方 MoveItPy Tutorial Suite 把 API 拆成六个组件练习：`MoveItPy`（顶层接口）、`PlanningSceneMonitor`（场景同步）、`RobotModel`/`RobotState`（状态与正运动学）、`PlanComponents`（规划请求）、`PoseGoal`/`JointGoal`（目标类型）与 `moveit_py` 的 display 工具。官方教程的关键启示是“目标类型决定规划维度”：`PlanComponents.setGoal(joint_values=...)` 构造全关节空间规划，而 `setGoal(pose=...)` 触发逆运动学后接笛卡尔校验；混合目标（部分关节 + 位姿）则走 `MotionPlanRequest` 的组合约束。练习第 3 题“命名位姿 vs 关节目标”的比较，官方教程直接用 `setGoal(named=...)` 与 `setGoal(joint_values=...)` 两行代码演示了等价关系。

## 27.2 PlanningComponent详解

### 27.2.1 创建PlanningComponent

```python
# 基本创建
self.arm = PlanningComponent(
    self.moveit, 'arm_group', 'link5'
)

# 获取多个规划组
self.arm = PlanningComponent(self.moveit, 'arm_group', 'link5')
self.gripper = PlanningComponent(self.moveit, 'gripper', 'gripper_link')
```

### 27.2.2 设置规划参数

```python
# 位置容忍度（米）
self.arm.set_goal_position_tolerance(0.01)

# 姿态容忍度（弧度）
self.arm.set_goal_orientation_tolerance(0.01)

# 参考坐标系
self.arm.set_pose_reference_frame('base_link')

# 末端执行器link
end_effector_link = self.arm.get_end_effector_link()

# 速度缩放因子 (0.0 ~ 1.0)
self.arm.set_max_velocity_scaling_factor(0.5)

# 加速度缩放因子 (0.0 ~ 1.0)
self.arm.set_max_acceleration_scaling_factor(0.5)
```

### 27.2.3 设置目标

```python
from geometry_msgs.msg import Pose, PoseStamped
from math import pi
from tf_transformations import quaternion_from_euler

# 方法1：关节空间目标
joint_goal = [0.5, -0.3, 0.2, 0.0, 0.0, 0.0]
self.arm.set_joint_value_target(joint_goal)

# 方法2：字典形式关节目标
joint_goal_dict = {
    'joint1': 0.5,
    'joint2': -0.3,
    'joint3': 0.2,
    'joint4': 0.0,
    'joint5': 0.0,
    'joint6': 0.0,
}
self.arm.set_joint_value_target(joint_goal_dict)

# 方法3：命名位姿
self.arm.set_named_target('home')

# 方法4：末端位姿目标
target_pose = Pose()
target_pose.position.x = 0.4
target_pose.position.y = 0.1
target_pose.position.z = 0.3
q = quaternion_from_euler(pi, 0, 0)
target_pose.orientation.x = q[0]
target_pose.orientation.y = q[1]
target_pose.orientation.z = q[2]
target_pose.orientation.w = q[3]
self.arm.set_pose_target(target_pose, 'link5')
```

### 27.2.4 规划与执行

```python
# 基本规划
plan_result = self.arm.plan()

# 指定规划器和规划时间
plan_result = self.arm.plan(
    planner_id='RRTConnectkConfigDefault',
    planning_time=5.0
)

# 检查规划结果
if plan_result:
    trajectory = plan_result.trajectory
    planning_time = plan_result.planning_time
    fraction = plan_result.fraction  # 笛卡尔路径覆盖率
    self.get_logger().info(
        f'规划成功: 轨迹点{len(trajectory.points)}, '
        f'耗时{planning_time:.2f}s'
    )
    # 执行轨迹（MoveItPy 中执行接口挂在 MoveItPy 对象上）
    self.moveit.execute(plan_result)
else:
    self.get_logger().error('规划失败')
```

> **执行前置条件**：`execute` 依赖 `/xarm_controller/follow_joint_trajectory`、`/gripper_controller/follow_joint_trajectory` 执行服务器，需先按第18章实验环境启动 `arm_only.launch.py`（含 `controller_manager`，即方式2）。课程统一封装 `course_lab_utils.moveit2` 中的 `ensure_execution_servers` 会在执行前对服务器做有界等待，服务器不可用时明确报错退出，而不是无界挂起。

### 27.2.5 获取当前状态

```python
from geometry_msgs.msg import Pose

# 获取当前关节值
joint_values = self.arm.get_current_joint_values()
self.get_logger().info(f'关节值: {joint_values}')

# 获取当前末端位姿
current_pose = self.arm.get_current_pose('link5')
if current_pose:
    pos = current_pose.pose.position
    self.get_logger().info(
        f'末端位置: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})'
    )
```

## 27.3 关节空间运动规划

### 27.3.1 正运动学规划

正运动学规划直接指定各关节的目标角度，MoveIt2规划一条从当前位置到目标位置的平滑轨迹。

```python
#!/usr/bin/env python3
"""关节空间运动规划示例"""
import time
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningComponent

class JointSpacePlanner(Node):
    def __init__(self):
        super().__init__('joint_space_planner')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.arm = PlanningComponent(
            self.moveit, 'arm_group', 'link5'
        )
        self.gripper = PlanningComponent(
            self.moveit, 'gripper', 'gripper_link'
        )

    def move_arm_to_joint(self, joint_positions):
        """移动到指定关节位置"""
        self.arm.set_start_state_to_current_state()
        self.arm.set_joint_value_target(joint_positions)

        plan_result = self.arm.plan()
        if plan_result:
            self.get_logger().info(f'执行关节目标: {joint_positions}')
            self.arm.execute(plan_result.trajectory)
            return True
        else:
            self.get_logger().warn(f'关节目标规划失败: {joint_positions}')
            return False

    def move_gripper(self, open=True):
        """控制夹爪开合"""
        goal = [0.65, 0.65] if open else [0.0, 0.0]
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_joint_value_target(goal)
        plan_result = self.gripper.plan()
        if plan_result:
            self.gripper.execute(plan_result.trajectory)

    def run_sequence(self):
        """执行一系列关节运动"""
        # 回到home位姿
        self.get_logger().info('回到home位姿')
        self.arm.set_named_target('home')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
        time.sleep(1)

        # 打开夹爪
        self.get_logger().info('打开夹爪')
        self.move_gripper(open=True)
        time.sleep(0.5)

        # 运动到第一个关节目标
        self.move_arm_to_joint([0.5, -0.3, 0.2, 0.0, 0.0, 0.0])
        time.sleep(1)

        # 运动到第二个关节目标
        self.move_arm_to_joint([-0.5, -0.5, 0.8, 0.0, 0.0, 0.0])
        time.sleep(1)

        # 关闭夹爪
        self.get_logger().info('关闭夹爪')
        self.move_gripper(open=False)
        time.sleep(0.5)

        # 回到home
        self.arm.set_named_target('home')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)

        self.get_logger().info('关节空间规划演示完成')

def main(args=None):
    rclpy.init(args=args)
    node = JointSpacePlanner()
    node.run_sequence()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 27.3.2 使用命名位姿

在SRDF中预先定义命名位姿，方便在程序中直接调用：

```python
# 定义命名位姿
self.arm.set_named_target('home')
self.arm.set_named_target('vertical')
self.arm.set_named_target('horizontal')
self.arm.set_named_target('retract')

# 规划并执行
plan = self.arm.plan()
if plan:
    self.arm.execute(plan.trajectory)
```

## 27.4 逆运动学求解

### 27.4.1 设置末端位姿目标

逆运动学规划指定末端执行器的目标位姿，MoveIt2解算出各关节角度后规划轨迹：

```python
#!/usr/bin/env python3
"""逆运动学规划示例"""
import time
from math import pi
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from moveit.planning import MoveItPy, PlanningComponent

class IKPlanner(Node):
    def __init__(self):
        super().__init__('ik_planner')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.arm = PlanningComponent(
            self.moveit, 'arm_group', 'link5'
        )

        # 配置规划参数
        end_effector_link = 'link5'
        reference_frame = 'base_link'

        self.arm.set_pose_reference_frame(reference_frame)
        self.arm.set_goal_position_tolerance(0.02)
        self.arm.set_goal_orientation_tolerance(0.03)
        self.arm.set_max_acceleration_scaling_factor(0.6)
        self.arm.set_max_velocity_scaling_factor(0.35)

    def plan_to_pose(self, x, y, z, roll=0, pitch=0, yaw=0):
        """规划到位姿目标"""
        # 先回到home
        self.arm.set_named_target('home')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
        time.sleep(1)

        # 构建目标位姿
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z

        q = quaternion_from_euler(roll, pitch, yaw)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        # 规划
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose.pose, 'link5')
        plan_result = self.arm.plan()

        if plan_result:
            self.get_logger().info(
                f'IK规划成功: 目标位置({x:.2f}, {y:.2f}, {z:.2f})'
            )
            self.arm.execute(plan_result.trajectory)
            return True
        else:
            self.get_logger().error(
                f'IK规划失败: 目标位置({x:.2f}, {y:.2f}, {z:.2f})'
            )
            return False

    def run(self):
        # 多个目标位姿
        targets = [
            (0.3, 0.1, 0.25, pi, 0, 0),
            (0.3, -0.1, 0.25, pi, 0, 0),
            (0.4, 0.0, 0.35, pi, 0, 0),
        ]

        for x, y, z, roll, pitch, yaw in targets:
            success = self.plan_to_pose(x, y, z, roll, pitch, yaw)
            if not success:
                self.get_logger().warn(f'目标 {x},{y},{z} 不可达')
            time.sleep(1)

        self.get_logger().info('IK规划演示完成')

def main(args=None):
    rclpy.init(args=args)
    node = IKPlanner()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 27.4.2 直接逆运动学求解

MoveItPy也支持直接求解逆运动学，不经过运动规划：

```python
from moveit.core.kinematics import KinematicsSolver

class DirectIKSolver(Node):
    def __init__(self):
        super().__init__('direct_ik_solver')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.robot_model = self.moveit.get_robot_model()
        self.kinematics_solver = KinematicsSolver(self.robot_model)

    def solve_ik(self, group_name, target_pose, link_name):
        """直接求解逆运动学"""
        robot_state = self.robot_model.get_default_state()

        success, solution = self.kinematics_solver.solve_ik(
            group_name=group_name,
            target_pose=target_pose,
            link_name=link_name,
            robot_state=robot_state,
            timeout=0.1,
        )

        if success:
            joint_positions = solution.get_joint_group_positions(group_name)
            self.get_logger().info(f'IK解算成功: {joint_positions}')
            return joint_positions
        else:
            self.get_logger().error('IK解算失败')
            return None
```

### 27.4.3 正运动学求解

```python
from moveit.core.kinematics import KinematicsSolver

class DirectFKSolver(Node):
    def __init__(self):
        super().__init__('direct_fk_solver')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.robot_model = self.moveit.get_robot_model()
        self.kinematics_solver = KinematicsSolver(self.robot_model)

    def solve_fk(self, group_name, joint_positions, link_name):
        """求解正运动学"""
        robot_state = self.robot_model.get_default_state()
        robot_state.set_joint_group_positions(group_name, joint_positions)
        robot_state.update()

        # 计算末端位姿
        pose = robot_state.get_pose(link_name)

        pos = pose.position
        self.get_logger().info(
            f'正运动学结果: 位置({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})'
        )
        return pose

    def print_current_pose(self):
        """输出当前末端位姿"""
        robot_state = self.robot_model.get_default_state()
        robot_state.update()
        pose = robot_state.get_pose('link5')
        pos = pose.position
        ori = pose.orientation
        self.get_logger().info(
            f'当前位置: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})\n'
            f'当前姿态: ({ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}, {ori.w:.3f})'
        )
```

### 27.4.4 官方要点——逆运动学官方接口的 IK 求解入口与回退

MoveItPy 对待 IK 的官方姿态是“把它当作规划目标而非独立数学函数”：`PlanComponents` 内部先查运动学求解器（默认 KDL 数值 IK，见第 24 章扩展的 kinematics.yaml 讨论），再执行可跳过 IK 的关节空间 fallback。官方示例仓库有一份独立的 `solveIK` 示例，演示了直接调用 `RobotState.setFromIK` 拿到多解并按评分排序的底层路径。练习第 2 题的“末端沿 Z 轴上升 0.1 米”本质是切比雪夫（Cartesian）问题：官方建议每次只给相对位姿偏移并重新求解，因为大幅位姿跳变容易落在奇异区或超出工作空间——这与第 24 章“多解性、存在性、奇异性”三个约束直接呼应。

### 27.4.5 官方要点——正运动学官方路径的 RobotState 查询式接口

练习第 5 题的正运动学在 MoveItPy 中由 `robot_state.getGlobalLinkTransform("gripper_centor_link")` 一行完成：官方教程说明它返回从规划基座到指定连杆的 `Isometry3d`，内部按 URDF 关节链实时解析，不持有任何缓存——因此读取时机（关节是否已更新）决定结果正确性。若要与底盘坐标系合并，官方文档要求先做 `setStateFromIK` 或用 `PlanningSceneMonitor` 的 TF 缓冲把链路接到全局帧。监控运行中状态的标准做法是订阅 `/joint_states`（ros2_control 发布）并同步到 RobotState，与本章仿真实例中的 `ros2 topic echo /joint_states` 一致。

## 27.5 完整应用示例

### 27.5.1 多目标运动规划器

```python
#!/usr/bin/env python3
"""多目标运动规划器"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy, PlanningComponent

class MultiTargetPlanner(Node):
    def __init__(self):
        super().__init__('multi_target_planner')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.arm = PlanningComponent(
            self.moveit, 'arm_group', 'link5'
        )

        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.01)

    def go_home(self):
        self.arm.set_named_target('home')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
            return True
        return False

    def plan_joint_motion(self, joint_goal, desc=''):
        self.arm.set_start_state_to_current_state()
        self.arm.set_joint_value_target(joint_goal)
        plan = self.arm.plan()
        if plan:
            self.get_logger().info(f'{desc}: 规划成功')
            self.arm.execute(plan.trajectory)
            return True
        self.get_logger().error(f'{desc}: 规划失败')
        return False

    def plan_cartesian_motion(self, pose_goal, desc=''):
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(pose_goal, 'link5')
        plan = self.arm.plan()
        if plan:
            self.get_logger().info(f'{desc}: 规划成功')
            self.arm.execute(plan.trajectory)
            return True
        self.get_logger().error(f'{desc}: 规划失败')
        return False

    def run_demo(self):
        self.go_home()
        time.sleep(0.5)

        # 关节空间运动
        self.plan_joint_motion(
            [0.5, -0.5, 0.3, 0.0, 0.0, 0.0],
            '关节运动1'
        )
        time.sleep(0.5)

        # 回到home
        self.go_home()
        time.sleep(0.5)

        # 笛卡尔空间运动
        pose = Pose()
        pose.position.x = 0.4
        pose.position.y = 0.0
        pose.position.z = 0.3
        pose.orientation.w = 1.0
        self.plan_cartesian_motion(pose, '笛卡尔运动1')
        time.sleep(0.5)

        self.get_logger().info('演示完成')

def main(args=None):
    rclpy.init(args=args)
    node = MultiTargetPlanner()
    node.run_demo()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 27.5.2 规划结果分析

```python
class PlanAnalyzer(Node):
    def analyze_plan(self, plan_result):
        """分析规划结果"""
        if not plan_result:
            return

        trajectory = plan_result.trajectory
        points = trajectory.points

        # 轨迹信息
        num_points = len(points)
        duration = points[-1].time_from_start.sec + \
                   points[-1].time_from_start.nanosec * 1e-9

        # 关节位置范围
        joint_ranges = []
        for j in range(len(points[0].positions)):
            positions = [p.positions[j] for p in points]
            joint_ranges.append(max(positions) - min(positions))

        self.get_logger().info(
            f'轨迹分析:\n'
            f'  轨迹点数: {num_points}\n'
            f'  总时长: {duration:.2f}s\n'
            f'  规划耗时: {plan_result.planning_time:.3f}s\n'
            f'  关节运动范围: {joint_ranges}'
        )
```

### 27.5.3 官方要点——规划失败排错的官方文档三步诊断

MoveIt 官方 Troubleshooting 页面对练习中常见的规划失败给出固定排查序列：第一步看规划日志中的 `Error Generating Plan, error code 1/2/3`——code 1 是初始状态越限（start state bounds，planning_adapters 的第一层），code 2 是目标不可达 / IK 无解，code 3 是规划器超时；第二步用 RViz 的 Planning Scene 面板勾选显示 start/goal 状态，目测目标是否在可行域；第三步用 `ros2 run moveit_ros_move_group move_group --ros-args` 打开 debug 日志看采样失败具体在哪些空间维。PickNik Academy 把三步做成了“十分钟排错”视频课；The Construct 的课程则用真实工程案例演示了 MoveItPy 与 RViz 配合的 observe-request-act 循环。

## 课后练习

1. 编写MoveItPy程序，控制六自由度机械臂依次运动到5个不同的关节目标位置，每次到达后输出当前关节值。

2. 使用逆运动学规划，让末端执行器沿Z轴上升0.1米、沿X轴移动0.2米、再回到初始位置。

3. 编写程序，分别使用命名位姿和关节目标两种方式控制机械臂运动到同一位置，比较代码复杂度和执行效果。

4. 使用MoveItPy的逆运动学求解器，计算末端位姿(x=0.3, y=0.1, z=0.2, roll=pi, pitch=0, yaw=0)对应的关节角度。

5. 编写正运动学程序，读取当前各关节角度，计算并输出末端执行器当前位姿。

---

## 仿真结合实例（当前仓库）：MoveItPy 关节目标与 xArm RViz 规划

### 目标与知识点对应

把本章 `PlanningComponent` 的关节空间目标接入 xArm6 的 MoveIt2 配置，先在 RViz 观察规划，再由 Python 程序读取规划结果和关节状态。

### 运行步骤

需要外部兼容的 `xarm_description` 2.0.0：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source /path/to/xarm_description_workspace/install/setup.bash
ros2 launch xarm_ros2_arm_only arm_only_move_group.launch.py use_rviz:=true
```

另开终端运行本章或实验中的 MoveItPy 节点，并在 RViz 中观察 `gripper_centor_link` 和规划轨迹：

```bash
source install/setup.bash
ros2 run moveit_fk_ik_lab fk_demo
ros2 topic echo /joint_states --once
```

### 源码与边界

MoveItPy 参考代码位于 `src/lab_code/ch17_lab/moveit_fk_ik_lab/`，xArm 配置位于 `src/xarm/config/`，MoveIt Launch 文件为 `src/xarm/launch/arm_only_move_group.launch.py`。

入口是否能执行取决于本地 `moveit_py` 和 xArm 描述依赖；不把未运行的规划写成成功结果。

![ch17 MoveIt 运动学规划运行输出](../lab_manuals/images/runtime/ch17_ik_demo.gif)

> 参考来源：
> - MoveIt 2 官方文档 —— MoveItPy Tutorial Suite 与 Troubleshooting：https://moveit.picknik.ai/
> - MoveIt 官方示例仓库 —— moveit_py 包：https://github.com/ros-planning/moveit2
> - PickNik Academy —— MoveItPy 培训课程：https://academy.picknik.ai/
> - The Construct —— MoveIt Python 课程：https://www.theconstructsim.com/
> - Robotics Back-End —— MoveItPy 实战教程：https://roboticsbackend.com/
