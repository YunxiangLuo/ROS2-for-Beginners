# 第12章 实验手册: MoveIt 2 机械臂编程

## 当前仓库仿真验证：xArm6 关节规划与状态读取

### 实验目标

把单关节规划、PlanningComponent 和 `/joint_states` 检查接入仓库的 xArm6 Gazebo/MoveIt2/RViz 环境。

### 运行步骤

需先准备兼容的外部 `xarm_description` 2.0.0：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source /path/to/xarm_description_workspace/install/setup.bash
ros2 launch xarm_ros2_arm_only arm_only.launch.py
```

在 RViz MotionPlanning 中选择 `xarm` 组设置关节目标，另开终端检查：

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

### 观察与边界

RViz 应显示目标状态和规划轨迹，Gazebo 显示机械臂模型。若外部描述包或控制器缺失，记录为环境前置条件失败，不应伪造运动成功。源码：`src/xarm/launch/`、`src/xarm/config/`。

## 环境说明
本实验使用 XBot-U 手臂 + Gazebo 仿真, 预装 moveit_py 和 ros-humble-moveit。

---

## 练习1: MoveIt2 配置加载与单轴运动 (~30 分钟)

### 目标
加载 MoveIt 2 配置, 掌握 PlanningComponent 规划与执行流程, 完成单关节运动。

### 步骤

#### 1.1 启动 MoveIt 2 仿真环境
```bash
# 终端1: 启动 Gazebo + 机械臂仿真
ros2 launch xbot_sim xbot_arm_gazebo.launch.py

# 终端2: 启动 MoveIt 2 (含 ros2_control)
ros2 launch xbot_moveit_config moveit.launch.py \
  use_sim_time:=true use_fake_hardware:=false

# 终端3: 启动 RViz
rviz2 -d src/courseware/rviz/moveit_view.rviz
```

#### 1.2 编写基础运动控制节点
```python
#!/usr/bin/env python3
"""练习1: MoveIt2 单轴运动与状态获取"""
import rclpy
from rclpy.node import Node
import numpy as np
import time


class BasicArmMover(Node):
    """基础机械臂运动控制器 (使用 MoveItPy)"""
    def __init__(self):
        super().__init__('basic_arm_mover')

        # 初始化 MoveItPy
        from moveit.planning import MoveItPy, PlanningComponent
        self.moveit = MoveItPy(node_name='moveit_py')
        self.arm = PlanningComponent(
            self.moveit, 'xbot_arm', 'xbot_gripper'
        )
        self.get_logger().info('MoveItPy 初始化完成')

    def get_current_joints(self) -> dict:
        """获取当前关节角"""
        state = self.moveit.get_robot_state()
        joint_names = [
            'arm_joint1', 'arm_joint2', 'arm_joint3',
            'arm_joint4', 'arm_joint5', 'arm_joint6'
        ]
        joints = {}
        for name in joint_names:
            joints[name] = state.get_joint_positions(name)
        return joints

    def get_current_ee_pose(self) -> list:
        """获取当前末端执行器位姿"""
        state = self.moveit.get_robot_state()
        ee_pose = state.get_pose('ee_link')
        p = ee_pose.position
        o = ee_pose.orientation
        return [p.x, p.y, p.z, o.x, o.y, o.z, o.w]

    def move_single_joint(self, joint_name: str, target_rad: float,
                          speed: float = 1.0):
        """
        控制单个关节运动
        参数:
            joint_name: 关节名称
            target_rad: 目标角度 (弧度)
            speed: 速度因子 (1.0 = 全速)
        """
        current = self.get_current_joints()
        if joint_name not in current:
            self.get_logger().error(f'未知关节: {joint_name}')
            return False

        # 设置目标 (仅改变指定关节)
        target = dict(current)
        target[joint_name] = target_rad

        self.get_logger().info(
            f'{joint_name}: {current[joint_name]:.3f} → {target_rad:.3f}'
        )

        # 规划
        self.arm.set_start_state_to_current_state()
        plan = self.arm.plan(
            target=target,
            planner_id='RRTConnectkConfigDefault',
            planning_time=2.0,
            max_velocity_scaling_factor=speed,
        )

        if plan:
            self.arm.execute(plan)
            self.get_logger().info(f'{joint_name} 运动完成!')
            return True
        else:
            self.get_logger().error(f'{joint_name} 规划失败!')
            return False

    def scan_joint_range(self, joint_name: str, start: float,
                         end: float, steps: int = 5):
        """
        扫描关节运动范围 (检验关节限制)
        参数:
            joint_name: 关节名称
            start, end: 扫描范围 (弧度)
            steps: 步数
        """
        self.get_logger().info(f'扫描关节 {joint_name} [{start:.2f}, {end:.2f}]')
        angles = np.linspace(start, end, steps)
        for i, angle in enumerate(angles):
            success = self.move_single_joint(joint_name, angle, speed=0.5)
            if not success:
                self.get_logger().warn(f'角度 {angle:.2f} 规划失败!')
                break
            time.sleep(1.5)

    def go_home(self):
        """回到 Home 位姿"""
        home = {
            'arm_joint1': 0.0,
            'arm_joint2': -0.5,
            'arm_joint3': 0.0,
            'arm_joint4': -1.5,
            'arm_joint5': 0.0,
            'arm_joint6': 1.0,
        }
        self.arm.set_start_state_to_current_state()
        plan = self.arm.plan(target=home, planner_id='RRTConnectkConfigDefault')
        if plan:
            self.arm.execute(plan)
            self.get_logger().info('已回到 Home 位姿')

    def run_demo(self):
        """演示: 逐个关节运动"""
        self.get_logger().info('=== 单关节运动演示 ===')

        # 回到 Home
        self.go_home()
        time.sleep(2.0)

        # 逐个关节运动
        self.move_single_joint('arm_joint1', 1.0)
        time.sleep(1.5)
        self.move_single_joint('arm_joint1', -1.0)
        time.sleep(1.5)
        self.move_single_joint('arm_joint2', -1.0)
        time.sleep(1.5)
        self.move_single_joint('arm_joint3', 1.0)
        time.sleep(1.5)
        self.move_single_joint('arm_joint4', -2.0)

        # 回到 Home
        self.go_home()
        ee_pose = self.get_current_ee_pose()
        self.get_logger().info(
            f'末端位姿: ({ee_pose[0]:.3f}, {ee_pose[1]:.3f}, {ee_pose[2]:.3f})'
        )


def main():
    rclpy.init()
    node = BasicArmMover()
    node.run_demo()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 验收标准
- [ ] MoveItPy 成功加载配置
- [ ] 各关节可独立规划并执行
- [ ] 能获取当前关节角和末端位姿
- [ ] Home 位姿正确恢复

---

## 练习2: 笛卡尔路径规划 (~30 分钟)

### 目标
掌握笛卡尔空间直线路径规划, 使用末端执行器画出指定形状。

### 步骤

#### 2.1 编写笛卡尔路径程序
```python
#!/usr/bin/env python3
"""练习2: 笛卡尔路径规划 — 画正方形/圆形"""
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningComponent
from geometry_msgs.msg import Pose
import numpy as np
import time
import copy


class CartesianPathPlanner(Node):
    """笛卡尔路径规划器"""
    def __init__(self):
        super().__init__('cartesian_planner')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.arm = PlanningComponent(self.moveit, 'xbot_arm', 'xbot_gripper')
        self.get_logger().info('笛卡尔规划器已就绪')

    def get_current_pose(self) -> Pose:
        """获取当前末端位姿"""
        state = self.moveit.get_robot_state()
        ee_pose = state.get_pose('ee_link')
        pose = Pose()
        pose.position = copy.deepcopy(ee_pose.position)
        pose.orientation = copy.deepcopy(ee_pose.orientation)
        return pose

    def compute_cartesian_path(self, waypoints: list[Pose],
                                step: float = 0.01,
                                jump_threshold: float = 2.0):
        """
        计算笛卡尔路径
        参数:
            waypoints: 目标位姿序列
            step: 插值分辨率 (m)
            jump_threshold: 关节跳变检测阈值
        """
        result = self.arm.compute_cartesian_path(
            waypoints=waypoints,
            step=step,
            jump_threshold=jump_threshold,
        )
        if result:
            # 检查路径完成度 (0~1)
            fraction = result.fraction
            self.get_logger().info(f'笛卡尔路径完成度: {fraction*100:.1f}%')
            return result if fraction > 0.99 else None
        return None

    def draw_square(self, side_length: float = 0.15,
                    plane_height: float = 0.3):
        """
        在水平面上画正方形
        参数:
            side_length: 边长 (m)
            plane_height: 绘图平面高度 (m)
        """
        self.get_logger().info(f'绘制 {side_length}m 正方形, 高度 {plane_height}m')

        # 移动到起点
        start_pose = self.get_current_pose()
        start_pose.position.x -= side_length / 2
        start_pose.position.y -= side_length / 2
        start_pose.position.z = plane_height

        plan = self.arm.plan(target=start_pose)
        if plan:
            self.arm.execute(plan)
        else:
            self.get_logger().error('无法到达起点')
            return

        # 正方形 4 条边
        edges = [
            ( side_length,  0, 0),           # 沿 X+
            ( 0,  side_length, 0),           # 沿 Y+
            (-side_length,  0, 0),           # 沿 X-
            ( 0, -side_length, 0),           # 沿 Y-
        ]

        for edge_idx, (dx, dy, dz) in enumerate(edges):
            self.get_logger().info(f'绘制边 {edge_idx+1}/4')

            edge_waypoints = []
            current = self.get_current_pose()
            n_steps = 20  # 每边 20 个路径点

            for i in range(1, n_steps + 1):
                t = i / n_steps
                wp = Pose()
                wp.position.x = current.position.x + dx * t
                wp.position.y = current.position.y + dy * t
                wp.position.z = current.position.z + dz * t
                wp.orientation = copy.deepcopy(current.orientation)
                edge_waypoints.append(wp)

            result = self.compute_cartesian_path(edge_waypoints)
            if result:
                self.arm.execute(result)
                self.get_logger().info(f'边 {edge_idx+1} 完成 ({n_steps} 个路径点)')
            else:
                self.get_logger().error(f'边 {edge_idx+1} 路径不可达')
                break

        self.get_logger().info('正方形绘制完成!')

    def draw_circle(self, radius: float = 0.1, plane_height: float = 0.3,
                    n_points: int = 72):
        """
        在水平面上画圆
        参数:
            radius: 半径 (m)
            plane_height: 绘图平面高度
            n_points: 路径点数
        """
        self.get_logger().info(f'绘制半径 {radius}m 圆, {n_points} 个点')

        # 移动到圆起点
        start = self.get_current_pose()
        start.position.x += radius
        start.position.z = plane_height

        plan = self.arm.plan(target=start)
        if plan:
            self.arm.execute(plan)
        else:
            return

        # 圆形路径点
        waypoints = []
        center = self.get_current_pose()
        cx, cy = center.position.x, center.position.y

        for i in range(1, n_points + 1):
            angle = 2 * np.pi * i / n_points
            wp = Pose()
            wp.position.x = cx + radius * np.cos(angle)
            wp.position.y = cy + radius * np.sin(angle)
            wp.position.z = plane_height
            wp.orientation = copy.deepcopy(center.orientation)
            waypoints.append(wp)

        result = self.compute_cartesian_path(waypoints)
        if result:
            self.arm.execute(result)
            self.get_logger().info('圆形绘制完成!')

    def run_demo(self):
        """演示: 画正方形 + 圆形"""
        # 先回到安全位置
        self.arm.set_start_state_to_current_state()
        home = {'arm_joint1': 0.0, 'arm_joint2': -0.5, 'arm_joint3': 0.0,
                'arm_joint4': -1.5, 'arm_joint5': 0.0, 'arm_joint6': 1.0}
        plan = self.arm.plan(target=home)
        if plan:
            self.arm.execute(plan)
        time.sleep(2.0)

        # 画正方形
        self.draw_square(side_length=0.15, plane_height=0.25)
        time.sleep(1.0)

        # 画圆
        self.draw_circle(radius=0.08, plane_height=0.25)
        time.sleep(1.0)

        # 回到 Home
        plan = self.arm.plan(target=home)
        if plan:
            self.arm.execute(plan)


def main():
    rclpy.init()
    node = CartesianPathPlanner()
    node.run_demo()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 验收标准
- [ ] 正方形 4 边连续画出, 无明显中断
- [ ] 圆形 72 个路径点完整执行
- [ ] 路径完成度 fraction > 99%
- [ ] 机械臂运动平稳, 无关节跳变

---

## 练习3: Pick & Place 抓取任务 (~30 分钟)

### 目标
实现完整的 Pick & Place 流程: 检测物体 → 规划抓取 → 抓取 → 移动到目标 → 放置。

### 步骤

#### 3.1 编写 Pick & Place 完整流程
```python
#!/usr/bin/env python3
"""练习3: Pick & Place 抓取放置任务"""
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningComponent
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
import numpy as np
import copy
import time


class PickPlaceRunner(Node):
    """Pick & Place 任务执行器"""
    def __init__(self):
        super().__init__('pick_place_runner')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.arm = PlanningComponent(self.moveit, 'xbot_arm', 'xbot_gripper')
        self.gripper = PlanningComponent(
            self.moveit, 'xbot_arm', 'xbot_gripper'
        )
        self.scene = self.moveit.get_planning_scene_monitor()

        # 物体参数
        self.object_name = 'target_cube'
        self.object_size = (0.04, 0.04, 0.04)  # 4cm 正方体
        self.get_logger().info('Pick & Place 执行器已就绪')

    def add_box_to_scene(self, name: str, size: tuple,
                         x: float, y: float, z: float):
        """在规划场景中添加碰撞物体"""
        obj = CollisionObject()
        obj.id = name
        obj.header.frame_id = 'world'

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(s) for s in size]
        obj.primitives.append(box)

        obj_pose = Pose()
        obj_pose.position.x = x
        obj_pose.position.y = y
        obj_pose.position.z = z
        obj_pose.orientation.w = 1.0
        obj.primitive_poses.append(obj_pose)
        obj.operation = CollisionObject.ADD

        self.scene.apply_collision_object(obj)
        self.get_logger().info(f'添加物体: {name} ({x:.2f}, {y:.2f}, {z:.2f})')

    def remove_object(self, name: str):
        """移除物体"""
        obj = CollisionObject()
        obj.id = name
        obj.operation = CollisionObject.REMOVE
        self.scene.apply_collision_object(obj)

    def attach_object(self, obj_name: str, link: str = 'gripper_link'):
        """将物体附着到夹爪 (模拟抓取)"""
        obj = AttachedCollisionObject()
        obj.object.id = obj_name
        obj.link_name = link
        obj.object.operation = CollisionObject.ADD
        self.scene.apply_attached_collision_object(obj)
        self.get_logger().info(f'物体 {obj_name} 已附着到 {link}')

    def detach_object(self, obj_name: str, link: str = 'gripper_link'):
        """从夹爪分离物体 (模拟放置)"""
        obj = AttachedCollisionObject()
        obj.object.id = obj_name
        obj.link_name = link
        obj.object.operation = CollisionObject.REMOVE
        self.scene.apply_attached_collision_object(obj)
        self.get_logger().info(f'物体 {obj_name} 已分离')

    def open_gripper(self):
        """打开夹爪"""
        self.get_logger().info('夹爪打开')
        target = {'gripper_joint': 0.035}  # 全开 0.035m
        plan = self.arm.plan(target=target)
        if plan:
            self.arm.execute(plan)

    def close_gripper(self):
        """关闭夹爪"""
        self.get_logger().info('夹爪关闭')
        target = {'gripper_joint': 0.0}  # 闭合
        plan = self.arm.plan(target=target)
        if plan:
            self.arm.execute(plan)

    def pick(self, obj_x: float, obj_y: float, obj_z: float):
        """
        抓取物体
        流程:
          1. 打开夹爪
          2. 移动到预抓取位姿 (物体上方)
          3. 笛卡尔下降到抓取位姿
          4. 关闭夹爪 (抓取)
          5. 附着物体到夹爪
          6. 笛卡尔提升
        """
        self.get_logger().info(f'开始抓取物体 at ({obj_x:.2f}, {obj_y:.2f}, {obj_z:.2f})')

        # 1. 打开夹爪
        self.open_gripper()
        time.sleep(1.0)

        # 2. 移动到预抓取位姿 (物体上方 8cm)
        pre_grasp = Pose()
        pre_grasp.position.x = obj_x
        pre_grasp.position.y = obj_y
        pre_grasp.position.z = obj_z + 0.08  # 物体上方
        pre_grasp.orientation.x = 0.0
        pre_grasp.orientation.y = 1.0
        pre_grasp.orientation.z = 0.0
        pre_grasp.orientation.w = 0.0  # 夹爪朝下

        self.arm.set_start_state_to_current_state()
        plan = self.arm.plan(target=pre_grasp)
        if plan:
            self.arm.execute(plan)
            self.get_logger().info('到达预抓取位姿')
        else:
            self.get_logger().error('预抓取位姿不可达!')
            return False

        time.sleep(1.0)

        # 3. 笛卡尔下降到抓取位姿
        grasp_waypoints = []
        for z in np.arange(obj_z + 0.08, obj_z + 0.01, -0.005):
            wp = Pose()
            wp.position.x = obj_x
            wp.position.y = obj_y
            wp.position.z = z
            wp.orientation = copy.deepcopy(pre_grasp.orientation)
            grasp_waypoints.append(wp)

        result = self.arm.compute_cartesian_path(
            waypoints=grasp_waypoints, step=0.005, jump_threshold=0.0
        )
        if result and result.fraction > 0.95:
            self.arm.execute(result)
            self.get_logger().info('到达抓取位姿')
        else:
            self.get_logger().error('抓取路径不可达!')
            return False

        # 4. 关闭夹爪
        self.close_gripper()
        time.sleep(1.0)

        # 5. 附着物体
        self.attach_object(self.object_name)
        time.sleep(0.5)

        # 6. 笛卡尔提升
        lift_waypoints = []
        for z in np.arange(obj_z + 0.01, obj_z + 0.15, 0.005):
            wp = Pose()
            wp.position.x = obj_x
            wp.position.y = obj_y
            wp.position.z = z
            wp.orientation = copy.deepcopy(pre_grasp.orientation)
            lift_waypoints.append(wp)

        result = self.arm.compute_cartesian_path(
            waypoints=lift_waypoints, step=0.005, jump_threshold=0.0
        )
        if result:
            self.arm.execute(result)
            self.get_logger().info('物体已提升')
            return True
        return False

    def place(self, target_x: float, target_y: float, target_z: float):
        """
        放置物体
        流程:
          1. 移动到放置预位置
          2. 笛卡尔下降到放置位姿
          3. 打开夹爪 (释放)
          4. 分离物体
          5. 笛卡尔提升
        """
        self.get_logger().info(f'开始放置物体 at ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})')

        # 1. 移动到放置预位置 (上方 8cm)
        pre_place = Pose()
        pre_place.position.x = target_x
        pre_place.position.y = target_y
        pre_place.position.z = target_z + 0.08
        pre_place.orientation.x = 0.0
        pre_place.orientation.y = 1.0
        pre_place.orientation.z = 0.0
        pre_place.orientation.w = 0.0

        self.arm.set_start_state_to_current_state()
        plan = self.arm.plan(target=pre_place)
        if plan:
            self.arm.execute(plan)
            self.get_logger().info('到达放置预位姿')
        else:
            self.get_logger().error('放置预位姿不可达!')
            return False

        time.sleep(1.0)

        # 2. 下降到放置位姿
        place_waypoints = []
        for z in np.arange(target_z + 0.08, target_z + 0.02, -0.005):
            wp = Pose()
            wp.position.x = target_x
            wp.position.y = target_y
            wp.position.z = z
            wp.orientation = copy.deepcopy(pre_place.orientation)
            place_waypoints.append(wp)

        result = self.arm.compute_cartesian_path(
            waypoints=place_waypoints, step=0.005, jump_threshold=0.0
        )
        if result:
            self.arm.execute(result)
            self.get_logger().info('到达放置位姿')
        else:
            self.get_logger().error('放置路径不可达!')
            return False

        # 3. 打开夹爪
        self.open_gripper()
        time.sleep(1.0)

        # 4. 分离物体 + 将物体添加回场景 (在放置位置)
        self.detach_object(self.object_name)
        self.add_box_to_scene(
            self.object_name + '_placed',
            self.object_size,
            target_x, target_y, target_z
        )
        time.sleep(0.5)

        # 5. 笛卡尔提升
        retreat_waypoints = []
        for z in np.arange(target_z + 0.02, target_z + 0.15, 0.005):
            wp = Pose()
            wp.position.x = target_x
            wp.position.y = target_y
            wp.position.z = z
            wp.orientation = copy.deepcopy(pre_place.orientation)
            retreat_waypoints.append(wp)

        result = self.arm.compute_cartesian_path(
            waypoints=retreat_waypoints, step=0.005, jump_threshold=0.0
        )
        if result:
            self.arm.execute(result)
            self.get_logger().info('放置完成, 机械臂已撤回')
        return True

    def go_home(self):
        """回到 Home 位姿"""
        home = {
            'arm_joint1': 0.0, 'arm_joint2': -0.5, 'arm_joint3': 0.0,
            'arm_joint4': -1.5, 'arm_joint5': 0.0, 'arm_joint6': 1.0,
        }
        self.arm.set_start_state_to_current_state()
        plan = self.arm.plan(target=home)
        if plan:
            self.arm.execute(plan)

    def run_demo(self):
        """执行演示"""
        # 初始化: 添加桌子 + 物体到场景
        self.add_box_to_scene('table', (0.8, 0.6, 0.05), 0.4, 0.0, 0.375)
        self.add_box_to_scene(
            self.object_name, self.object_size, 0.5, -0.1, 0.42
        )

        # 回到 Home
        self.go_home()
        time.sleep(2.0)

        # Pick: 抓取物体
        success = self.pick(0.5, -0.1, 0.43)
        if not success:
            self.get_logger().error('Pick 失败!')
            return

        time.sleep(1.5)

        # Place: 放置到新位置
        success = self.place(0.3, 0.2, 0.42)
        if not success:
            self.get_logger().error('Place 失败!')
            return

        # 回到 Home
        self.go_home()
        self.get_logger().info('Pick & Place 任务完成!')


def main():
    rclpy.init()
    node = PickPlaceRunner()
    node.run_demo()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 验收标准
- [ ] 成功抓取物体 (夹爪闭合到位)
- [ ] 物体在规划场景中随夹爪移动 (附着/分离正确)
- [ ] 成功放置到目标位置
- [ ] 机械臂回到 Home (无自碰撞)
- [ ] 笛卡尔路径完成度 > 95%

---

## 练习 4：MoveItPy 单关节运动规划（约 15 分钟）

### 目标
使用 moveit_py Python API 规划单个关节的运动，控制 XBot-U 机械臂完成指定角度旋转。

### 步骤

**步骤1：编写 single_joint_planner.py**
```python
#!/usr/bin/env python3
"""练习4: MoveItPy 单关节规划"""
import rclpy
import time


def main():
    rclpy.init()
    from moveit.planning import MoveItPy, PlanningComponent

    moveit = MoveItPy(node_name='moveit_py')
    arm = PlanningComponent(moveit, 'xbot_arm', 'xbot_gripper')

    print('规划 arm_joint1 旋转到 1.0 rad...')
    arm.set_start_state_to_current_state()
    plan = arm.plan(
        target={'arm_joint1': 1.0},
        planner_id='RRTConnectkConfigDefault',
        planning_time=2.0,
    )
    if plan:
        arm.execute(plan)
        print('运动完成!')
        time.sleep(1.0)

        print('规划 arm_joint1 回到 0.0 rad...')
        arm.set_start_state_to_current_state()
        plan = arm.plan(
            target={'arm_joint1': 0.0},
            planner_id='RRTConnectkConfigDefault',
        )
        if plan:
            arm.execute(plan)
            print('回零完成!')
    rclpy.shutdown()
```

**步骤2：启动 MoveIt 并运行**
```bash
ros2 launch xbot_moveit_config moveit.launch.py use_sim_time:=true
python3 single_joint_planner.py
```

**✓ 验证**：机械臂 arm_joint1 旋转到 1.0 rad 后再回到 0.0 rad，Gazebo/RViz 中可见运动。

### 思考题
1. `PlanningComponent.plan()` 返回的 plan 对象包含哪些信息？
2. 如果目标关节角超出限位，规划会怎样？
