# 第12章 MoveIt 2 机械臂编程

## 仿真结合实例（当前仓库）：xArm6 的 MoveIt2 规划场景

### 目标与知识点对应

使用仓库的 `xarm_ros2_arm_only` 启动 Gazebo、ros2_control、MoveIt2 和 RViz，观察规划组、关节状态和运动规划流程，对应本章的规划场景与执行器概念。

### 运行步骤

`xArm` 依赖外部兼容的 `xarm_description` 2.0.0。先确认该依赖已被 source，然后在工作区根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source /path/to/xarm_description_workspace/install/setup.bash
ros2 launch xarm_ros2_arm_only arm_only.launch.py
```

若只检查 MoveIt 配置：

```bash
ros2 launch xarm_ros2_arm_only arm_only_move_group.launch.py use_rviz:=true
```

在 RViz 的 MotionPlanning 面板中选择 `xarm` 规划组，拖动目标状态并执行 `Plan`；另开终端检查：

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

### 观察结果

RViz 显示 xArm 模型、规划组和轨迹；Gazebo 中的模型状态应由控制器反馈。若底层描述包缺失，启动会在解析 Xacro 前失败，应先补齐依赖。

### 源码与边界

- 启动：`src/xarm/launch/arm_only.launch.py`、`arm_only_move_group.launch.py`
- MoveIt 配置：`src/xarm/config/`
- Xacro 封装：`src/xarm/urdf/arm_only_xarm.urdf.xacro`

当前仓库不包含真实硬件驱动；本实例只针对仿真规划，不代表真实机械臂可用。

## 12.1 知识要点

### 12.1.1 MoveIt 2 架构概览

MoveIt 2 是 ROS 2 最主流的机械臂运动规划框架。

```
┌─────────────────────────────────────────────┐
│                MoveIt 2 架构                  │
│                                               │
│  ┌──────────┐     ┌─────────────────────┐    │
│  │ 用户接口  │────>│  MoveGroupInterface │    │
│  │(C++/Python)│   │   (核心 API)        │    │
│  └──────────┘     └─────────┬───────────┘    │
│                              │                │
│        ┌─────────────────────┼────────────┐  │
│        │                     │             │  │
│  ┌─────▼──────┐  ┌──────────▼───┐  ┌─────▼───┐
│  │ 运动规划器  │  │ 碰撞检测    │  │运动学   │
│  │ OMPL,CHOMP │  │ FCL,Bullet  │  │ KDL/TRAC │
│  └─────┬──────┘  └──────────┘   │ /IK      │
│        │                         │  │        │
│  ┌─────▼──────────────────────────▼──┐      │
│  │   规划场景 (Planning Scene)       │      │
│  │   - 机器人模型 (URDF/SRDF)        │      │
│  │   - 环境物体                      │      │
│  │   - 碰撞对象                      │      │
│  └───────────────────┬───────────────┘      │
│                      │                       │
│            ┌─────────▼─────────┐            │
│            │  ros2_control     │            │
│            │  (硬件接口)        │            │
│            └───────────────────┘            │
└─────────────────────────────────────────────┘
```

**核心概念:**
- **MoveGroup:** 机械臂/末端执行器的抽象接口
- **Planning Scene:** 完整的环境模型 (机器人 + 障碍物 + 物体)
- **Motion Planner:** 运动规划算法库
- **Kinematics Solver:** 正/逆运动学求解器

### 12.1.2 MoveItPy Python API

MoveItPy 是 MoveIt 2 的 Python 封装:

```python
#!/usr/bin/env python3
"""
MoveItPy 基础示例 — 机械臂运动控制
需要: pip install moveit, ros-humble-moveit-py
"""
import rclpy
from rclpy.node import Node

# MoveItPy 核心导入
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanningComponent,
    SinglePlanParameters,
    MultiPipelinePlanRequestParameters,
)


class MoveItPyArmController(Node):
    """基于 MoveItPy 的机械臂控制器"""
    def __init__(self):
        super().__init__('arm_controller')

        # 初始化 MoveItPy (加载配置)
        self.moveit = MoveItPy(node_name='moveit_py')
        # 获取 PlanningComponent (对应 MoveGroup)
        self.arm = PlanningComponent(
            self.moveit, 'panda_arm', 'panda_hand'
        )
        self.get_logger().info('MoveItPy 初始化完成')

    def plan_joint_goal(self, joint_values: list):
        """
        规划到指定关节角
        参数:
            joint_values: [j1, j2, j3, j4, j5, j6, j7] 关节角 (弧度)
        """
        # 创建关节目标
        self.arm.set_start_state_to_current_state()
        joint_goal = {f'panda_joint{i+1}': joint_values[i] for i in range(7)}

        # 规划
        plan_result = self.arm.plan(
            target=joint_goal,
            planner_id='RRTConnectkConfigDefault',
        )

        if plan_result:
            self.get_logger().info(f'规划成功! 轨迹点数: {len(plan_result.trajectory)}')
            return plan_result
        else:
            self.get_logger().error('规划失败!')
            return None

    def plan_pose_goal(self, x: float, y: float, z: float,
                       qx: float, qy: float, qz: float, qw: float):
        """
        规划到指定位姿 (逆运动学)
        参数:
            x, y, z: 位置 (m)
            qx, qy, qz, qw: 姿态四元数
        """
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header.frame_id = 'panda_link0'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.arm.set_start_state_to_current_state()
        plan_result = self.arm.plan(
            target=pose.pose,
            planner_id='RRTConnectkConfigDefault',
            constraints=None,
            planning_time=5.0,
        )

        if plan_result:
            self.get_logger().info(
                f'位姿规划成功! 时间: {plan_result.planning_time:.3f}s'
            )
        return plan_result

    def plan_cartesian_path(self, waypoints: list, step: float = 0.01,
                            jump_threshold: float = 0.0):
        """
        笛卡尔空间路径规划 (直线路径)
        参数:
            waypoints: 路径点列表 [(x0,y0,z0,q0), ...]
            step: 插值步长 (m)
            jump_threshold: 关节空间跳变阈值, 0 表示禁用
        返回: 轨迹或 None
        """
        self.arm.set_start_state_to_current_state()
        plan_result = self.arm.compute_cartesian_path(
            waypoints=waypoints,
            step=step,
            jump_threshold=jump_threshold,
        )
        return plan_result

    def execute(self, plan_result):
        """执行规划结果"""
        if plan_result:
            self.arm.execute(plan_result)
            self.get_logger().info('轨迹执行完成!')

    def stop(self):
        """停止机械臂运动"""
        self.arm.stop()

    def go_home(self):
        """回到 Home 位姿"""
        home_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        plan = self.plan_joint_goal(home_joints)
        if plan:
            self.execute(plan)

    def pick_and_place_demo(self):
        """
        Pick and Place 完整流程
        1. 移动到预抓取位置 (pre-grasp)
        2. 下降到抓取位置 (grasp)
        3. 抓取物体
        4. 提升 (lift)
        5. 移动到放置位置 (place)
        6. 释放物体
        """
        # 1. 预抓取位置 (目标上方 10cm)
        pre_grasp = self.plan_pose_goal(
            0.5, 0.0, 0.3,  # x, y, z
            0.0, 1.0, 0.0, 0.0  # 夹爪朝下
        )
        if pre_grasp:
            self.execute(pre_grasp)

        # 2. 笛卡尔下降到抓取位置
        import numpy as np
        cartesian_waypoints = []
        for z in np.arange(0.3, 0.1, -0.01):
            cartesian_waypoints.append([0.5, 0.0, z, 0.0, 1.0, 0.0, 0.0])

        if self.move_to_waypoints:
            cart_path = self.plan_cartesian_path(cartesian_waypoints)
            if cart_path:
                self.execute(cart_path)

        # 3. 抓取物体 (闭合夹爪)
        self.attach_object('target_object')

        # 4. 笛卡尔提升
        cartesian_lift = []
        for z in np.arange(0.1, 0.3, 0.01):
            cartesian_lift.append([0.5, 0.0, z, 0.0, 1.0, 0.0, 0.0])
        lift_path = self.plan_cartesian_path(cartesian_lift)
        if lift_path:
            self.execute(lift_path)

        # 5. 移动到放置位置
        place_pose = self.plan_pose_goal(
            -0.3, -0.5, 0.3,
            0.0, 1.0, 0.0, 0.0
        )
        if place_pose:
            self.execute(place_pose)

        # 6. 释放物体
        self.detach_object('target_object')

        # 7. 回到 Home
        self.go_home()
```

### 12.1.3 运动规划器

**OMPL (Open Motion Planning Library):**
```python
class OMPLPlannerConfig:
    """
    OMPL 规划器配置说明
    MoveIt 2 默认使用 OMPL 作为规划后端
    """
    # 常用 OMPL 规划器
    planners = {
        'RRTConnect':        '双向 RRT 连接, 快速, 非最优',
        'RRTstar':           'RRT* 渐进最优, 较慢',
        'PRM':               '概率路图, 适合多次查询',
        'PRMstar':           'PRM* 渐进最优',
        'EST':               '扩展空间树',
        'SBL':               '基于采样的均衡树',
        'KPIECE':            '动力学空间下的投影扩展',
        'TRRT':              '过渡态 RRT, 适合狭窄通道',
        'BiTRRT':            '双向 TRRT',
    }

    # 配置示例 (ompl_planning.yaml)
    default_config = """
    RRTConnectkConfigDefault:
      type: geometric::RRTConnect
      range: 0.0  # 0 = 无限制
    """

# STOMP (Stochastic Trajectory Optimization for Motion Planning)
# 基于优化的规划器, 通过随机噪声优化轨迹平滑性和避障
class STOMPPlanner:
    """
    STOMP: 随机轨迹优化
    - 不需要显式碰撞检测 (代价函数内处理)
    - 适合平滑的关节空间轨迹
    """
    pass

# CHOMP (Covariant Hamiltonian Optimization for Motion Planning)
class CHOMPPlanner:
    """
    CHOMP: 协变哈密顿优化
    - 基于梯度的轨迹优化
    - 同时优化平滑性和障碍物距离
    """
    pass
```

### 12.1.4 碰撞检测与规划场景

```python
from moveit.core.planning_scene import PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject


class SceneManager:
    """规划场景管理 — 添加/移除碰撞物体"""
    def __init__(self, moveit_instance):
        self.moveit = moveit_instance
        self.scene = PlanningScene(moveit_instance)

    def add_box(self, name: str, size: tuple, pose: tuple,
                frame_id: str = 'world'):
        """
        添加碰撞盒子
        参数:
            name: 物体名称
            size: (x, y, z) 尺寸 (m)
            pose: (x, y, z, qx, qy, qz, qw) 位姿
        """
        obj = CollisionObject()
        obj.id = name
        obj.header.frame_id = frame_id

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(s) for s in size]
        obj.primitives.append(primitive)

        obj_pose = Pose()
        obj_pose.position.x = pose[0]
        obj_pose.position.y = pose[1]
        obj_pose.position.z = pose[2]
        obj_pose.orientation.x = pose[3]
        obj_pose.orientation.y = pose[4]
        obj_pose.orientation.z = pose[5]
        obj_pose.orientation.w = pose[6]
        obj.primitive_poses.append(obj_pose)
        obj.operation = CollisionObject.ADD

        self.scene.apply_collision_object(obj)
        print(f'碰撞物体 {name} 已添加')

    def remove_object(self, name: str):
        """移除碰撞物体"""
        obj = CollisionObject()
        obj.id = name
        obj.operation = CollisionObject.REMOVE
        self.scene.apply_collision_object(obj)

    def attach_to_end_effector(self, obj_name: str):
        """将物体附着在末端执行器上 (模拟抓取)"""
        obj = AttachedCollisionObject()
        obj.object.id = obj_name
        obj.link_name = 'panda_hand'  # 末端 link
        obj.object.operation = CollisionObject.ADD
        self.scene.apply_attached_collision_object(obj)

    def detach_from_end_effector(self, obj_name: str):
        """从末端执行器分离物体"""
        obj = AttachedCollisionObject()
        obj.object.id = obj_name
        obj.object.operation = CollisionObject.REMOVE
        self.scene.apply_attached_collision_object(obj)

    def check_self_collision(self) -> bool:
        """检测自碰撞"""
        current_state = self.moveit.get_robot_state()
        collision_result = self.scene.check_self_collision(current_state)
        return collision_result.collision

    def check_collision_with_objects(self) -> bool:
        """检测与场景物体的碰撞"""
        current_state = self.moveit.get_robot_state()
        collision_result = self.scene.check_collision(
            current_state, verbose=True
        )
        if collision_result.collision:
            for contact in collision_result.contacts:
                print(f'碰撞: {contact.body_1} <-> {contact.body_2}')
        return collision_result.collision

    def add_table_and_shelf(self):
        """添加典型场景: 桌子 + 货架"""
        # 地面桌子 (80cm x 60cm)
        self.add_box(
            'table',
            (0.8, 0.6, 0.05),
            (0.5, 0.0, 0.35, 0.0, 0.0, 0.0, 1.0)
        )
        # 货架隔板
        self.add_box(
            'shelf_board',
            (0.8, 0.02, 0.6),
            (0.5, 0.3, 0.3, 0.0, 0.0, 0.0, 1.0)
        )
```

### 12.1.5 运动学求解器

```python
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink


class IKSolverDemo:
    """
    正运动学 (FK) / 逆运动学 (IK) 求解示例
    使用 ikpy 库 (教学参考)
    """
    def __init__(self):
        # 构建 7-DOF Panda 手臂链
        self.chain = Chain(name='panda', links=[
            OriginLink(),
            URDFLink(name='joint1', origin_translation=[0, 0, 0.333],
                     origin_orientation=[0, 0, 0], rotation=[0, 0, 1]),
            URDFLink(name='joint2', origin_translation=[0, 0, 0],
                     origin_orientation=[0, 0, 0], rotation=[0, 1, 0]),
            URDFLink(name='joint3', origin_translation=[0, 0, 0.316],
                     origin_orientation=[0, 0, 0], rotation=[0, 0, 1]),
            URDFLink(name='joint4', origin_translation=[0.0825, 0, 0],
                     origin_orientation=[0, 0, 0], rotation=[0, 1, 0]),
            URDFLink(name='joint5', origin_translation=[-0.0825, 0, 0.384],
                     origin_orientation=[0, 0, 0], rotation=[0, 0, 1]),
            URDFLink(name='joint6', origin_translation=[0, 0, 0],
                     origin_orientation=[0, 0, 0], rotation=[0, 1, 0]),
            URDFLink(name='joint7', origin_translation=[0.088, 0, 0],
                     origin_orientation=[0, 0, 0], rotation=[0, 0, 1]),
        ])

    def forward_kinematics(self, joints: list) -> np.ndarray:
        """
        正运动学: 关节角 → 末端位姿
        参数:
            joints: 7 个关节角 (弧度)
        返回:
            (4,4) 齐次变换矩阵
        """
        fk = self.chain.forward_kinematics(joints)
        return fk

    def inverse_kinematics(self, target_position: list,
                           target_orientation: list = None,
                           initial_joints: list = None) -> np.ndarray:
        """
        逆运动学: 目标位姿 → 关节角
        参数:
            target_position: [x, y, z] 目标位置
            target_orientation: [rx, ry, rz] 目标姿态 (轴角)
            initial_joints: 初始猜测关节角
        返回:
            7 个关节角 (弧度)
        """
        if initial_joints is None:
            initial_joints = [0.0] * 7
        ik = self.chain.inverse_kinematics(
            target_position,
            target_orientation,
            initial_position=initial_joints
        )
        return ik

    def test_reachable(self, x: float, y: float, z: float):
        """测试目标位置是否在可达空间内"""
        try:
            ik = self.inverse_kinematics([x, y, z])
            fk = self.forward_kinematics(ik.tolist())
            pos_err = np.linalg.norm(fk[:3, 3] - np.array([x, y, z]))
            return True, pos_err, ik
        except Exception:
            return False, float('inf'), None
```

### 12.1.6 ros2_control 接口集成

```python
#!/usr/bin/env python3
"""
ros2_control 控制器切换与管理
"""
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import (
    SwitchController, ListControllers, LoadController
)
from controller_manager_msgs.msg import ControllerState


class ControllerManager(Node):
    """ros2_control 控制器管理器"""
    def __init__(self):
        super().__init__('controller_manager_client')
        self.switch_srv = self.create_client(
            SwitchController, '/controller_manager/switch_controller'
        )
        self.list_srv = self.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )

    async def list_controllers(self) -> list:
        """列出所有控制器及状态"""
        future = self.list_srv.call_async(ListControllers.Request())
        result = await future
        for ctrl in result.controller:
            self.get_logger().info(
                f'{ctrl.name}: {ctrl.state} (类型: {ctrl.type})'
            )
        return result.controller

    async def switch_to_position_controller(self):
        """
        切换到位置控制器 (轨迹执行用)
        MoveIt 需要 joint_trajectory_controller
        """
        req = SwitchController.Request()
        req.activate_controllers = ['joint_trajectory_controller']
        req.deactivate_controllers = ['joint_state_broadcaster']
        req.strictness = SwitchController.Request.STRICT
        future = self.switch_srv.call_async(req)
        result = await future
        self.get_logger().info(f'控制器切换结果: {result.ok}')


def launch_control_setup():
    """
    MoveIt 2 + ros2_control 完整启动流程
    """
    commands = """
    # 1. 启动机器人描述和硬件接口
    ros2 launch robot_description display.launch.py use_fake_hardware:=true

    # 2. 启动 MoveIt 2
    ros2 launch moveit2 moveit.launch.py

    # 3. 启动控制器管理
    ros2 run controller_manager spawner joint_state_broadcaster
    ros2 run controller_manager spawner joint_trajectory_controller

    # 4. 启动 Python 运动控制节点
    python3 arm_control_demo.py
    """
    print(commands)
```

### 12.1.7 抓取规划

```python
import numpy as np


class GraspPlanner:
    """
    抓取规划器 — 生成候选抓取位姿
    参考: moveit_msgs/Grasp 消息定义
    """
    def __init__(self, obj_center: np.ndarray, obj_size: np.ndarray):
        """
        参数:
            obj_center: 物体中心 [x, y, z]
            obj_size: 物体尺寸 [sx, sy, sz]
        """
        self.obj_center = obj_center
        self.obj_size = obj_size

    def generate_top_grasps(self, n: int = 8) -> list:
        """
        生成顶部抓取候选
        在物体上方生成 n 个均匀分布方向的抓取姿态
        """
        grasps = []
        for i in range(n):
            angle = 2 * np.pi * i / n
            # 抓取位姿: 在物体上方, 夹爪朝下
            grasp = {
                'position': [
                    self.obj_center[0],
                    self.obj_center[1],
                    self.obj_center[2] + self.obj_size[2] / 2 + 0.05,
                ],
                'orientation': self._grasp_orientation(angle),
                'pre_grasp_approach': [
                    0, 0, -0.05  # 沿 -Z 接近
                ],
                'post_grasp_retreat': [
                    0, 0, 0.05   # 沿 +Z 撤回
                ],
            }
            grasps.append(grasp)
        return grasps

    def generate_side_grasps(self, n: int = 8) -> list:
        """
        生成侧面抓取候选
        """
        grasps = []
        for i in range(n):
            angle = 2 * np.pi * i / n
            dx = np.cos(angle) * (self.obj_size[0] / 2 + 0.1)
            dy = np.sin(angle) * (self.obj_size[1] / 2 + 0.1)
            grasp = {
                'position': [
                    self.obj_center[0] + dx,
                    self.obj_center[1] + dy,
                    self.obj_center[2],
                ],
                'orientation': self._side_grasp_orientation(angle),
                'pre_grasp_approach': [
                    -np.cos(angle) * 0.1, -np.sin(angle) * 0.1, 0
                ],
            }
            grasps.append(grasp)
        return grasps

    def _grasp_orientation(self, yaw: float) -> list:
        """生成夹爪朝向四元数"""
        import math
        return [0.0, math.cos(yaw), 0.0, math.sin(yaw)]

    def _side_grasp_orientation(self, angle: float) -> list:
        """生成侧面抓取朝向"""
        import math
        half = math.pi / 4
        return [0.0, math.sin(half), 0.0, math.cos(half)]
```

### 12.1.8 MoveIt 2 配置与启动

```yaml
# moveit_controller.yaml (控制器配置)
controller_names:
  - joint_trajectory_controller

joint_trajectory_controller:
  type: "FollowJointTrajectory"
  joints:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6
    - joint7
  action_monitor_rate: 20
  constraints:
    goal_time: 0.6
    stopped_velocity_tolerance: 0.05
    joint1: {trajectory: 0.1, goal: 0.1}
    joint2: {trajectory: 0.1, goal: 0.1}
    joint3: {trajectory: 0.1, goal: 0.1}
```

---

## 12.2 练习题

**1. 原理解析题:** 说明 MoveIt 2 中 Planning Scene、MoveGroup 和 Motion Planner 三者之间的关系。为什么需要维护一个 Planning Scene 而不是每次规划时直接读取传感器数据?

**2. 运动学题:** 给定 Panda 机械臂的 DH 参数, 手工推导正运动学 (FK) 表达式, 并编写 Python 代码实现 7-DOF 机械臂的正运动学计算。

**3. 编程题:** 使用 MoveItPy 编写一个 "画正方形" 程序: 末端执行器在水平面上绘制边长 0.2m 的正方形 (笛卡尔空间轨迹), 要求 4 条边各用 20 个路径点。

**4. 碰撞题:** 在规划场景中添加 3 个随机位置/尺寸的障碍物盒子, 编写程序验证机械臂从 A 点到 B 点的规划能否正确避开所有障碍物, 并输出碰撞检测结果。

**5. 设计题:** 设计一个 "传送带分拣" 场景: 传送带上有红色和蓝色两种物体, 机械臂需要将红色物体分拣到左侧箱子、蓝色物体分拣到右侧箱子。写出完整的程序框架 (包括视觉感知接口定义)。

**6. 综合题:** 比较 OMPL-RRTConnect、STOMP 和 CHOMP 三种规划器的优缺点和适用场景。针对焊接任务 (需要平滑轨迹且靠近工件) 应选择哪种规划器? 说明理由。
