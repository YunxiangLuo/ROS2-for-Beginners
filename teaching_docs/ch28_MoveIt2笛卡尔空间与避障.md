# 第28章 MoveIt2笛卡尔空间与避障

> **课程**：ROS2 Python 编程  
> **章节**：第28章  
> **课时**：2 课时（90 分钟）  
> **教学方式**：讲授 + 演示  

---

## 学习目标

本章学习目标包括：掌握笛卡尔路径规划的方法；理解位姿约束和路径约束的概念；学会在规划场景中添加障碍物；掌握避障规划的配置和调试。

## 28.1 笛卡尔路径规划

### 28.1.1 直线轨迹规划

笛卡尔空间规划要求机械臂末端沿指定路径运动，MoveIt2使用`plan_cartesian_path`函数计算笛卡尔路径。

MoveIt2实现笛卡尔规划的关键API是`plan_cartesian_path(waypoints, eef_step, jump_threshold, avoid_collisions)`，它根据给定的路径点列表计算笛卡尔路径：`waypoints`为路径点列表，`eef_step`为末端步进值（米），`jump_threshold`为跳跃阈值（0为不允许跳跃），`avoid_collisions`控制是否进行避障规划，返回值`(plan, fraction)`分别为规划结果和路径覆盖率。

```python
#!/usr/bin/env python3
"""笛卡尔直线路径规划示例"""
import time
from copy import deepcopy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit.planning import MoveItPy, PlanningComponent

class CartesianLinePlanner(Node):
    def __init__(self):
        super().__init__('cartesian_line_planner')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.arm = PlanningComponent(
            self.moveit, 'arm_group', 'link5'
        )

        reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(reference_frame)
        self.arm.set_goal_position_tolerance(0.005)
        self.arm.set_goal_orientation_tolerance(0.005)
        end_effector_link = 'link5'

        # 先回到home
        self.arm.set_named_target('home')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
        time.sleep(1)

        # 移动到一个初始位置
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.pose.position.x = 0.4
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.35
        target_pose.pose.orientation.w = 1.0

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose.pose, end_effector_link)
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
        time.sleep(1)

        # 构建笛卡尔路径点
        current_pose = self.arm.get_current_pose(end_effector_link).pose
        start_pose = deepcopy(current_pose)

        waypoints = [deepcopy(start_pose)]

        # 路径点1：向下移动0.1米
        wpose = deepcopy(start_pose)
        wpose.position.z -= 0.1
        waypoints.append(deepcopy(wpose))

        # 路径点2：向右移动0.1米
        wpose.position.y += 0.1
        waypoints.append(deepcopy(wpose))

        # 路径点3：向上移动0.1米
        wpose.position.z += 0.1
        waypoints.append(deepcopy(wpose))

        # 路径点4：回到起点
        waypoints.append(deepcopy(start_pose))

        # 执行笛卡尔规划
        fraction = 0.0
        maxtries = 100
        attempts = 0

        self.arm.set_start_state_to_current_state()
        while fraction < 1.0 and attempts < maxtries:
            plan_result = self.arm.plan_cartesian_path(
                waypoints, 0.01, 0.0, True
            )
            if plan_result:
                fraction = plan_result.fraction
            attempts += 1
            if attempts % 10 == 0:
                self.get_logger().info(
                    f'尝试 {attempts}/{maxtries}, 覆盖率: {fraction:.1%}'
                )

        if fraction >= 1.0:
            self.get_logger().info('笛卡尔路径规划成功')
            self.arm.execute(plan_result.trajectory)
        else:
            self.get_logger().error(f'规划失败: 覆盖率 {fraction:.1%}')

        # 回到home
        self.arm.set_named_target('home')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)

        self.get_logger().info('笛卡尔直线规划完成')

def main(args=None):
    rclpy.init(args=args)
    node = CartesianLinePlanner()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 28.1.2 圆弧轨迹规划

`plan_cartesian_path`支持直线路径，但可以通过微分思想实现圆弧轨迹：将圆弧切分为多个小线段，用线段近似圆弧。

```python
#!/usr/bin/env python3
"""圆弧轨迹规划示例"""
import math
import time
import numpy as np
from copy import deepcopy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanningComponent

class CartesianArcPlanner(Node):
    def __init__(self):
        super().__init__('cartesian_arc_planner')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.arm = PlanningComponent(
            self.moveit, 'arm_group', 'link5'
        )

        reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(reference_frame)
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)

        end_effector_link = 'link5'

        # 回到home
        self.arm.set_named_target('home')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
        time.sleep(1)

        # 移动到圆弧中心位置
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.pose.position.x = 0.4
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.35
        target_pose.pose.orientation.w = 1.0

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose.pose, end_effector_link)
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
        time.sleep(1)

        # 生成圆弧路径点（Y-Z平面）
        waypoints = []
        center_y = target_pose.pose.position.y
        center_z = target_pose.pose.position.z
        radius = 0.1

        for th in np.arange(0, math.pi * 2, 0.015):
            target_pose.pose.position.y = center_y + radius * math.cos(th)
            target_pose.pose.position.z = center_z + radius * math.sin(th)
            waypoints.append(deepcopy(target_pose.pose))

        # 执行笛卡尔规划
        fraction = 0.0
        maxtries = 100
        attempts = 0

        self.arm.set_start_state_to_current_state()
        while fraction < 1.0 and attempts < maxtries:
            plan_result = self.arm.plan_cartesian_path(
                waypoints, 0.01, 0.0, True
            )
            if plan_result:
                fraction = plan_result.fraction
            attempts += 1

        if fraction >= 1.0:
            self.get_logger().info('圆弧路径规划成功')
            self.arm.execute(plan_result.trajectory)
        else:
            self.get_logger().error(f'规划失败: 覆盖率 {fraction:.1%}')

        time.sleep(1)

        # 回到home
        self.arm.set_named_target('home')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)

        self.get_logger().info('圆弧规划完成')

def main(args=None):
    rclpy.init(args=args)
    node = CartesianArcPlanner()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 28.1.3 S形曲线规划

除了直线和圆弧，还可以生成S形、锯齿形等复杂路径：

```python
def generate_s_shape(start_pose, amplitude=0.05, steps=20):
    """生成S形路径点"""
    waypoints = []
    for i in range(steps):
        t = i / steps
        x = start_pose.position.x + t * 0.1  # X方向前进
        y = start_pose.position.y + amplitude * math.sin(t * 4 * math.pi)
        z = start_pose.position.z

        pose = deepcopy(start_pose)
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        waypoints.append(deepcopy(pose))
    return waypoints

def generate_zigzag(start_pose, width=0.08, segments=4):
    """生成锯齿形路径点"""
    waypoints = []
    direction = 1
    for i in range(segments):
        for j in range(10):
            t = j / 10
            x = start_pose.position.x + (i * 0.02 + t * 0.02)
            y = start_pose.position.y + direction * width * t

            pose = deepcopy(start_pose)
            pose.position.x = x
            pose.position.y = y
            waypoints.append(deepcopy(pose))
        direction *= -1
    return waypoints
```

### 28.1.4 官方要点——官方笛卡尔路径教程：computeCartesianPath 的三个参数

> 本节内容综合翻译自 MoveIt 2 官方文档（moveit.picknik.ai 的 Cartesian Path、Planning Scene 与 Motion Planning Pipeline 教程）、OMPL 官方文档（ompl.kavrakilab.org）与 PickNik Academy 培训课程，另参考 The Construct 的 MoveIt 避障课程与 Robotics Back-End 的规划场景实战教程。原文均为英文，此处为中文编译，供课后巩固与进阶阅读。

MoveIt 官方 Cartesian Path Tutorial 明确了 `computeCartesianPath` 的三个决定性参数：`eef_step`（末端插值步长，官方建议 0.01 m 量级，过大会跳过碰撞）、`jump_threshold`（关节空间跳变阈值，用于拒绝解臂形变附近的"假直线"，设 0 表示禁用检测）与 `avoid_collisions`（是否在插值中逐点做碰撞检查）。官方特别强调返回值 fraction（成功插值的 waypoint 比例）必须达到 1.0（或显式接受的阈值）才可执行——练习第 1 题的"正方形四边"要用四次直线笛卡尔段而非一个 waypoint 列表直接到位，每次转弯处规划器会自然停顿重算。

## 28.2 位姿约束

### 28.2.1 路径约束类型

MoveIt2支持在规划过程中添加各种约束，确保路径满足特定要求：

| 约束类型 | 说明 | 适用场景 |
|---------|------|---------|
| PositionConstraint | 位置约束 | 限制末端位置范围 |
| OrientationConstraint | 姿态约束 | 保持末端姿态 |
| JointConstraint | 关节约束 | 限制特定关节角度 |
| VisibilityConstraint | 可见性约束 | 目标必须在视野内 |

### 28.2.2 添加位姿约束

```python
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Point

class ConstrainedPlanner(Node):
    def __init__(self):
        super().__init__('constrained_planner')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.arm = PlanningComponent(
            self.moveit, 'arm_group', 'link5'
        )

    def add_position_constraint(self):
        """添加位置约束：限制末端在指定区域内"""
        constraints = Constraints()
        constraints.name = "position_constraint"

        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name = 'link5'
        pc.weight = 1.0

        # 定义约束区域（球体）
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.3]  # 半径0.3米的球体

        pc.constraint_region.primitives = [sphere]
        pc.constraint_region.primitive_poses = [Pose()]
        pc.constraint_region.primitive_poses[0].position.x = 0.3
        pc.constraint_region.primitive_poses[0].position.y = 0.0
        pc.constraint_region.primitive_poses[0].position.z = 0.2

        constraints.position_constraints = [pc]

        self.arm.set_path_constraints(constraints)
        self.get_logger().info('位置约束已添加')

    def add_orientation_constraint(self):
        """添加姿态约束：保持末端朝下"""
        constraints = Constraints()
        constraints.name = "orientation_constraint"

        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'link5'
        oc.weight = 1.0

        # 目标姿态：末端朝下（绕X轴旋转180度）
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(math.pi, 0, 0)
        oc.orientation.x = q[0]
        oc.orientation.y = q[1]
        oc.orientation.z = q[2]
        oc.orientation.w = q[3]

        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1

        constraints.orientation_constraints = [oc]
        self.arm.set_path_constraints(constraints)
        self.get_logger().info('姿态约束已添加')

    def clear_constraints(self):
        """清除所有约束"""
        self.arm.clear_path_constraints()
        self.get_logger().info('约束已清除')
```

### 28.2.3 关节约束

```python
from moveit_msgs.msg import JointConstraint

def add_joint_constraint(self, joint_name, target_value, tolerance=0.1):
    """添加关节约束：限制某个关节的角度范围"""
    jc = JointConstraint()
    jc.joint_name = joint_name
    jc.position = target_value
    jc.tolerance_above = tolerance
    jc.tolerance_below = tolerance
    jc.weight = 1.0

    constraints = Constraints()
    constraints.joint_constraints = [jc]
    self.arm.set_path_constraints(constraints)
    self.get_logger().info(f'关节 {joint_name} 约束已添加: {target_value}±{tolerance}')
```

### 28.2.4 官方要点——位姿约束与圆弧轨迹的官方做法

对练习第 2 题的圆弧轨迹，官方文档给出的标准方案是"密集 waypoint + 位姿约束"：在应用层按参数方程生成圆弧上的采样位姿（每段 0.005~0.01 rad），逐次调用笛卡尔规划；或改用 `MotionPlanRequest` 的 `path_constraints`（方向约束 tolerance）让优化型规划器在连续空间内保持姿态。官方文档提醒：笛卡尔路径是"直线插值 + 校验"而非"带约束的求解"，因此圆弧越细越接近真实弧线——这也是 The Construct 课程中"画圆"练习的核心结论。

## 28.3 规划场景与障碍物

### 28.3.1 PlanningSceneInterface

MoveIt2通过规划场景接口提供场景更新API，包括添加、移除、附着物体等操作。

MoveIt2规划场景涉及三个关键概念：**CollisionObject**（规划场景中的碰撞物体，包括盒体、球体、圆柱体与网格模型）、**AttachedCollisionObject**（附着在机械臂上的物体）与**PlanningScene**（完整的规划场景消息）。

### 28.3.2 添加障碍物

```python
#!/usr/bin/env python3
"""规划场景与障碍物示例"""
import time
from math import pi
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene, ObjectColor, CollisionObject
from shape_msgs.msg import SolidPrimitive
from moveit.planning import MoveItPy, PlanningComponent

class ObstacleDemo(Node):
    def __init__(self):
        super().__init__('obstacle_demo')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()
        self.scene_pub = self.create_publisher(
            PlanningScene, 'planning_scene', 10
        )
        time.sleep(1)

        self.arm = PlanningComponent(
            self.moveit, 'arm_group', 'link5'
        )
        self.arm.set_goal_position_tolerance(0.02)
        self.arm.set_goal_orientation_tolerance(0.03)

        # 回到home
        self.arm.set_named_target('home')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)

    def add_box(self, object_id, frame_id, x, y, z, size):
        """添加长方体障碍物"""
        co = CollisionObject()
        co.id = object_id
        co.header.frame_id = frame_id
        co.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = size

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        co.primitives = [primitive]
        co.primitive_poses = [pose.pose]
        self.planning_scene_monitor.process_collision_object(co)

    def add_sphere(self, object_id, frame_id, x, y, z, radius):
        """添加球体障碍物"""
        co = CollisionObject()
        co.id = object_id
        co.header.frame_id = frame_id
        co.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [radius]

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        co.primitives = [primitive]
        co.primitive_poses = [pose.pose]
        self.planning_scene_monitor.process_collision_object(co)

    def remove_object(self, object_id):
        """移除障碍物"""
        co = CollisionObject()
        co.id = object_id
        co.operation = CollisionObject.REMOVE
        self.planning_scene_monitor.process_collision_object(co)

    def add_color(self, object_id, r, g, b, a=1.0):
        """设置物体颜色"""
        color = ObjectColor()
        color.id = object_id
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a

        p = PlanningScene()
        p.is_diff = True
        p.object_colors = [color]
        self.scene_pub.publish(p)

    def run(self):
        # 添加桌子
        self.get_logger().info('添加桌子')
        self.add_box('table', 'base_link', 0.0, 0.0, -0.005, [1.0, 1.2, 0.01])
        self.add_color('table', 0.6, 0.4, 0.2)
        time.sleep(1)

        # 添加障碍物球体
        self.get_logger().info('添加球体障碍物')
        self.add_sphere('sphere_obs', 'base_link', 0.3, 0.15, 0.12, 0.12)
        self.add_color('sphere_obs', 0.8, 0.0, 0.9)
        time.sleep(1)

        # 添加障碍物盒体
        self.get_logger().info('添加盒体障碍物')
        self.add_box('box_obs', 'base_link', 0.3, -0.15, 0.2, [0.25, 0.05, 0.4])
        self.add_color('box_obs', 0.8, 0.8, 0.0)
        time.sleep(1)

        # 规划避障路径
        self.get_logger().info('规划避障路径')
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = 0.25
        target_pose.pose.position.y = -0.35
        target_pose.pose.position.z = 0.25

        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(pi, 0, 0)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose.pose, 'link5')
        plan_result = self.arm.plan()

        if plan_result:
            self.get_logger().info('避障规划成功')
            self.arm.execute(plan_result.trajectory)
        else:
            self.get_logger().error('避障规划失败')

        time.sleep(2)

        # 移除障碍物
        self.remove_object('table')
        self.remove_object('sphere_obs')
        self.remove_object('box_obs')
        time.sleep(1)

        self.get_logger().info('障碍物演示完成')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDemo()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 28.3.3 物体附着与分离

当机械臂抓取物体后，需要将物体附着到末端执行器上，使MoveIt2在规划时考虑该物体的碰撞体积：

```python
from moveit_msgs.msg import AttachedCollisionObject

class AttachDetachDemo(Node):
    def __init__(self):
        super().__init__('attach_detach_demo')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.psm = self.moveit.get_planning_scene_monitor()

    def attach_object(self, object_id, link_name):
        """将物体附着到指定的连杆上"""
        co = CollisionObject()
        co.id = object_id
        co.header.frame_id = link_name
        co.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.04, 0.04, 0.1]

        pose = PoseStamped()
        pose.header.frame_id = link_name
        pose.pose.position.z = 0.05
        pose.pose.orientation.w = 1.0

        co.primitives = [primitive]
        co.primitive_poses = [pose.pose]

        aco = AttachedCollisionObject()
        aco.link_name = link_name
        aco.object = co
        aco.touch_links = ['link5', 'gripper_link']

        self.psm.process_attached_collision_object(aco)
        self.get_logger().info(f'物体 {object_id} 已附着到 {link_name}')

    def detach_object(self, object_id, link_name):
        """从连杆上分离物体"""
        co = CollisionObject()
        co.id = object_id
        co.operation = CollisionObject.REMOVE

        aco = AttachedCollisionObject()
        aco.link_name = link_name
        aco.object = co

        self.psm.process_attached_collision_object(aco)
        self.get_logger().info(f'物体 {object_id} 已从 {link_name} 分离')
```

### 28.3.4 官方要点——规划场景官方 API：物体、附着与八叉树

MoveIt 官方 Planning Scene Tutorial 把 26.3 节的 API 归纳为四个操作：`applyCollisionObjects`（批量添加 box/sphere/cylinder/mesh，含位姿与颜色）、`moveCollisionObject`（平移已存在物体）、`attachObject`（把物体挂到某个连杆，随末端运动）与 `detachObject`（释放回场景）。练习第 4 题的"附着前后差异"在官方教程中有精确对应：附着后物体从环境碰撞体变为机器人的一部分，自碰撞检查会把它与机械臂的接触视为合法（通过 Allowed Collision Matrix 自动放行），因此抓取后的工作空间行为会显著变化。真实深度相机环境则走 octomap 通道（`octomap` 服务自动更新，可用 `octocollision_object` 过滤），本章仿真实例的 `/planning_scene` topic 就是官方标准的监控接口。

## 28.4 避障规划

### 28.4.1 避障规划策略

MoveIt2的避障规划基于以下策略：**碰撞检测**（在规划过程中持续检测机器人与障碍物的碰撞）、**配置空间搜索**（在无碰撞的配置空间中搜索路径）、**安全余量**（通过padding参数设置碰撞安全距离）与**重规划**（环境变化时自动重新规划）。

### 28.4.2 碰撞安全余量设置

```yaml
# 在MoveIt配置中设置碰撞安全余量
collision:
  padding: 0.02       # 全局碰撞安全余量 2cm
  padding_scale: 1.0  # 缩放因子
  max_contacts: 10    # 最大碰撞接触点数
```

编程方式设置：

```python
from moveit.core.collision_detection import (
    CollisionRequest, CollisionResult, AllowedCollisionMatrix
)

class SafetyConfig:
    def set_safety_margins(self, planning_scene_monitor, padding=0.02):
        """设置碰撞安全余量"""
        acm = AllowedCollisionMatrix(
            planning_scene_monitor.get_planning_scene().get_robot_model()
        )
        # 可以设置特定连杆对的安全余量
        # acm.set_entry('link1', 'link2', True)  # True=允许碰撞
        return acm

    def check_collision_with_margin(self, planning_scene, robot_state, margin=0.02):
        """带安全余量的碰撞检测"""
        request = CollisionRequest()
        request.padding = margin
        request.contacts = True

        result = CollisionResult()
        planning_scene.check_collision(request, result, robot_state)

        return result
```

### 28.4.3 避障规划完整示例

```python
#!/usr/bin/env python3
"""避障规划完整示例"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanningComponent

class ObstacleAvoidancePlanner(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_planner')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.psm = self.moveit.get_planning_scene_monitor()
        self.arm = PlanningComponent(
            self.moveit, 'arm_group', 'link5'
        )

    def setup_environment(self):
        """设置规划环境"""
        co = CollisionObject()
        co.id = 'table'
        co.header.frame_id = 'base_link'
        co.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.8, 0.6, 0.02]
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.z = -0.01
        pose.pose.orientation.w = 1.0
        co.primitives = [primitive]
        co.primitive_poses = [pose.pose]
        self.psm.process_collision_object(co)

        # 添加左侧障碍物
        co2 = CollisionObject()
        co2.id = 'left_obstacle'
        co2.header.frame_id = 'base_link'
        co2.operation = CollisionObject.ADD

        prim2 = SolidPrimitive()
        prim2.type = SolidPrimitive.BOX
        prim2.dimensions = [0.1, 0.1, 0.3]
        pose2 = PoseStamped()
        pose2.header.frame_id = 'base_link'
        pose2.pose.position.x = 0.3
        pose2.pose.position.y = -0.2
        pose2.pose.position.z = 0.15
        pose2.pose.orientation.w = 1.0
        co2.primitives = [prim2]
        co2.primitive_poses = [pose2.pose]
        self.psm.process_collision_object(co2)

        # 添加右侧障碍物
        co3 = CollisionObject()
        co3.id = 'right_obstacle'
        co3.header.frame_id = 'base_link'
        co3.operation = CollisionObject.ADD

        prim3 = SolidPrimitive()
        prim3.type = SolidPrimitive.BOX
        prim3.dimensions = [0.1, 0.1, 0.3]
        pose3 = PoseStamped()
        pose3.header.frame_id = 'base_link'
        pose3.pose.position.x = 0.3
        pose3.pose.position.y = 0.2
        pose3.pose.position.z = 0.15
        pose3.pose.orientation.w = 1.0
        co3.primitives = [prim3]
        co3.primitive_poses = [pose3.pose]
        self.psm.process_collision_object(co3)

        self.get_logger().info('环境设置完成：桌子 + 2个障碍物')

    def plan_avoidance(self):
        """规划避障路径（从中间穿过）"""
        # 回到home
        self.arm.set_named_target('home')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
        time.sleep(1)

        # 目标位姿（在障碍物后方）
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = 0.4
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.25

        # 尝试多次规划
        for attempt in range(5):
            self.get_logger().info(f'避障规划尝试 {attempt+1}')
            self.arm.set_start_state_to_current_state()
            self.arm.set_pose_target(target_pose.pose, 'link5')

            # 增加规划时间以提高成功率
            plan_result = self.arm.plan(
                planner_id='RRTConnectkConfigDefault',
                planning_time=5.0
            )

            if plan_result:
                self.get_logger().info('避障规划成功')
                self.arm.execute(plan_result.trajectory)
                return True
            time.sleep(0.5)

        self.get_logger().error('避障规划失败，目标可能不可达')
        return False

    def cleanup(self):
        """清理环境"""
        for obj in ['table', 'left_obstacle', 'right_obstacle']:
            co = CollisionObject()
            co.id = obj
            co.operation = CollisionObject.REMOVE
            self.psm.process_collision_object(co)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidancePlanner()
    node.setup_environment()
    node.plan_avoidance()
    time.sleep(2)
    node.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 28.4.4 规划场景的可视化

在Rviz中配置MotionPlanning插件，可以实时查看规划场景：

```
Rviz MotionPlanning 插件配置：
- PlanningScene Topic: /planning_scene
- Robot Description: robot_description
- Planning Group: arm_group
- Show Robot Visual: True
- Show Robot Collision: True
- Scene Geometry: Show All
```

### 28.4.5 官方要点——狭窄通道：OMPL 官方的规划器建议

练习第 5 题的狭窄通道是 OMPL 官方文档反复强调的经典难题（narrow passage problem）：RRTConnect 在开阔空间几乎必成，但两侧树都难以穿过窄缝；PRM/EST 这类基于"采样扩散"的算法在通道内布点效率更低，官方文档建议在受限空间中改用带目标的采样策略或对称连接（Symmetric Connect）类规划器，并配合 `link_padding`/`link_scale`（26.4 节的安全余量）做"缩放通过"的调参——先用小 padding 规划成功再逐步加大验证。PickNik Academy 的避障课程演示了用 benchmark 工具把成功率-时间曲线量化成表格的方法，与本章练习要求的"比较成功率"完全一致。

## 课后练习

1. 编写笛卡尔路径规划程序，使末端执行器沿正方形的四条边运动（边长0.1米），回到起点。

2. 编写圆弧轨迹规划程序，使末端在水平面内画一个完整的圆形。

3. 在规划场景中添加三个不同形状的障碍物（盒体、球体、圆柱体），规划一条从起点到终点的避障路径。

4. 编写程序，将物体附着到末端执行器后规划运动，再分离物体，比较附着前后的规划行为差异。

5. 设计一个包含狭窄通道的避障场景，测试不同规划器（RRTConnect、PRM、EST）通过狭窄通道的成功率。

---

## 仿真结合实例（当前仓库）：xArm RViz 规划场景中的笛卡尔路径

### 目标与知识点对应

使用 xArm6 的 MoveIt2/RViz 规划场景，添加碰撞物体并用本章的笛卡尔路径代码生成末端 waypoint，观察路径约束和碰撞检测的作用。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source /path/to/xarm_description_workspace/install/setup.bash
ros2 launch xarm_ros2_arm_only arm_only_move_group.launch.py use_rviz:=true
```

在 RViz MotionPlanning 中打开 Planning Scene，添加盒体障碍物；将本章示例的规划组改为 `xarm`、末端改为 `gripper_centor_link`，再观察规划轨迹和碰撞结果。

```bash
ros2 topic echo /display_planned_path --once
ros2 topic echo /planning_scene --once
```

### 源码与边界

实验参考代码位于`src/lab_code/ch18_lab/moveit_pick_place_lab/`，xArm 规划配置位于`src/xarm/config/arm_only_ompl_planning.yaml`，RViz 配置位于`src/xarm/config/arm_only_moveit.rviz`。

仓库没有预置可保证成功的障碍物场景；结果以本地 RViz 和 MoveIt 返回值为准。

![ch18 MoveIt 路径跟随运行输出](../lab_manuals/images/runtime/ch18_beeline_demo.gif)


> 参考来源：
> - MoveIt 2 官方文档 —— Cartesian Path、Planning Scene 与 Motion Planning Pipeline 教程：https://moveit.picknik.ai/
> - OMPL 官方文档 —— 规划器与 benchmark：https://ompl.kavrakilab.org/
> - PickNik Academy —— MoveIt 官方培训课程：https://academy.picknik.ai/
> - The Construct —— MoveIt 避障课程：https://www.theconstructsim.com/
> - Robotics Back-End —— 规划场景实战教程：https://roboticsbackend.com/
