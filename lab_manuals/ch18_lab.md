# 第18章 实验：MoveIt2 规划场景、避障与抓取放置

## 当前仓库仿真验证：xArm 规划场景与碰撞物体

### 实验目标

使用 xArm6 MoveIt2/RViz 添加碰撞物体，验证笛卡尔路径、避障规划以及 attach/detach 场景更新。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source /path/to/xarm_description_workspace/install/setup.bash
ros2 launch xarm_ros2_arm_only arm_only.launch.py
```

在 RViz MotionPlanning 中选择 `xarm`，添加盒体障碍物并运行：

```bash
source install/setup.bash
ros2 topic echo /planning_scene --once
ros2 topic echo /display_planned_path --once
```

### 观察与边界

观察障碍物加入前后的规划轨迹以及 attach/detach 后的场景状态。源码：`src/lab_code/ch18_lab/moveit_pick_place_lab/`、`src/xarm/config/`。仓库没有保证成功的完整抓取物体和执行证据，结果以本地 MoveIt 返回值为准。

> **对应理论章节**：第28章《笛卡尔路径与避障》、第29章《抓取与放置实验》
> **实验课时**：4课时  
> **实验代码**：`src/lab_code/ch18_lab/moveit_pick_place_lab/`  

## 实验目标
- 掌握笛卡尔空间直线路径规划方法
- 学会使用`plan_cartesian_path`进行连续路径规划
- 理解碰撞检测与避障原理
- 能够在规划场景中添加障碍物并验证避障效果
- 掌握MoveItPy中物体的attach和detach操作
- 实现完整的Pick-and-Place流程
- 理解规划场景中的碰撞物体管理
- 学会使用PlanningSceneMonitor进行场景同步

## 实验环境
- ROS 2 Jazzy + MoveIt2
- RViz2 + MotionPlanning插件

**MoveIt 前置依赖（统一说明）**：本章实验开始前，需先启动 xArm 仿真与 MoveIt：

```bash
# 方式1: 纯 MoveIt + RViz 仿真（不含 Gazebo）
ros2 launch xarm_ros2_arm_only arm_only_move_group.launch.py

# 方式2: 含 Gazebo 的完整仿真（本章参考代码 README 使用的命令）
ros2 launch xarm_ros2_arm_only arm_only.launch.py
```

默认使用上述 `xarm_ros2_arm_only` 启动命令。若按第17章17.1节自行生成配置包，请使用该包的实际名称及其生成的 launch 文件。

**方式限定**：方式1 不启动 `controller_manager`，仅适用于规划演示与 RViz 轨迹观察。本章的执行型程序（`beeline_demo`、`obstacles_demo`、`attach_object_demo`、`pick_place_demo`）需要真实执行轨迹，请使用方式2。若在方式1下运行执行型程序，程序会等待执行服务器并在超时后报错退出（提示启动 `arm_only.launch.py`）。

## 参考代码说明
`src/lab_code/ch18_lab/moveit_pick_place_lab/`（ament_python 包）提供以下程序：

| 程序 | 功能 |
|------|------|
| `beeline_demo.py` | 笛卡尔直线路径规划。沿三角形路径的三个顶点做笛卡尔直线运动 |
| `obstacles_demo.py` | 障碍物避障演示。在规划场景中添加桌子、球体和长方体，观察机械臂避障运动 |
| `attach_object_demo.py` | 物体附着与分离。将工具附着到末端执行器，执行抓取放置 |
| `pick_place_demo.py` | 抓取-放置（Pick & Place）全流程综合演示 |
| `target_publisher.py` | 3D目标位姿发布器。发布正弦运动的目标点（`PoseStamped` 到 `/target_pose`），可在 RViz 中可视化 |

构建：
```bash
cd <工作区根目录>
colcon build --symlink-install --packages-select moveit_pick_place_lab
source install/setup.bash
```

## 18.1 笛卡尔路径与避障

### 18.1.1 运行参考代码
启动 xArm 仿真（见前置说明）后，另开终端运行：

```bash
ros2 run moveit_pick_place_lab beeline_demo       # 直线路径规划
ros2 run moveit_pick_place_lab obstacles_demo     # 避障演示（按Enter逐步执行）
ros2 run moveit_pick_place_lab target_publisher   # 3D目标点发布
```

`target_publisher.py` 支持参数调整：
```bash
python3 target_publisher.py --ros-args \
  -p rate:=20 \
  -p speed:=1.5 \
  -p target_frame:=base_link
```
可在 RViz 中添加 `Marker` 和 `PoseStamped` 显示查看动态移动的目标球体。

### 18.1.2 创建实验包
```bash
cd ~/ros2_arm_ws/src
ros2 pkg create ch18_cartesian --build-type ament_python --dependencies rclpy moveit
cd ch18_cartesian
mkdir -p ch18_cartesian
```

### 18.1.3 编写直线路径规划程序
创建 `ch18_cartesian/line_path.py`:
```python
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningComponent
from geometry_msgs.msg import Pose
from copy import deepcopy
import time

class LinePathPlanner(Node):
    def __init__(self):
        super().__init__('line_path_planner')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.arm = PlanningComponent(
            self.moveit, 'xarm_group', 'gripper_centor_link'
        )
        self.reference_frame = 'base_link'
        self.end_effector = 'gripper_centor_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.get_logger().info('直线路径规划器已启动')

    def get_current_pose(self):
        return self.arm.get_current_pose(self.end_effector).pose

    def move_to_pose(self, target_pose):
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector)
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
            return True
        return False

    def plan_cartesian_line(self, waypoints):
        self.arm.set_start_state_to_current_state()
        fraction = 0.0
        max_tries = 100
        attempts = 0

        while fraction < 1.0 and attempts < max_tries:
            plan_result = self.arm.plan_cartesian_path(
                waypoints, 0.01, 0.0, True
            )
            if plan_result:
                fraction = plan_result.fraction
                self.get_logger().info(
                    f'尝试 {attempts+1}, 覆盖率: {fraction*100:.1f}%'
                )
            attempts += 1

        if fraction >= 1.0:
            self.get_logger().info('笛卡尔路径规划成功!')
            self.arm.execute(plan_result.trajectory)
            return True
        else:
            self.get_logger().error(f'规划失败, 最大覆盖率: {fraction*100:.1f}%')
            return False

    def run_straight_line(self):
        self.get_logger().info('=== 直线路径演示 ===')

        start_pose = Pose()
        start_pose.position.x = 0.3
        start_pose.position.y = 0.0
        start_pose.position.z = 0.45
        start_pose.orientation.w = 1.0

        self.get_logger().info('移动到起始点')
        if not self.move_to_pose(start_pose):
            self.get_logger().error('起始点不可达')
            return
        time.sleep(1)

        waypoints = []
        current = self.get_current_pose()

        wpose = deepcopy(current)
        wpose.position.z += 0.1
        waypoints.append(deepcopy(wpose))

        wpose.position.y += 0.15
        waypoints.append(deepcopy(wpose))

        wpose.position.z -= 0.1
        waypoints.append(deepcopy(wpose))

        wpose.position.y -= 0.15
        waypoints.append(deepcopy(wpose))

        self.get_logger().info('规划直线路径...')
        self.plan_cartesian_line(waypoints)

def main(args=None):
    rclpy.init(args=args)
    node = LinePathPlanner()
    node.run_straight_line()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 18.1.4 编写方形路径规划程序
创建 `ch18_cartesian/square_path.py`:
```python
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningComponent
from geometry_msgs.msg import Pose
from copy import deepcopy
import time

class SquarePathPlanner(Node):
    def __init__(self):
        super().__init__('square_path_planner')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.arm = PlanningComponent(
            self.moveit, 'xarm_group', 'gripper_centor_link'
        )
        self.arm.set_pose_reference_frame('base_link')
        self.end_effector = 'gripper_centor_link'

    def run_square(self, side_length=0.12, height=0.35):
        self.get_logger().info(f'画方形: 边长{side_length}m, 高度{height}m')

        start_pose = Pose()
        start_pose.position.x = 0.3
        start_pose.position.y = -side_length/2
        start_pose.position.z = height
        start_pose.orientation.w = 1.0

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(start_pose, self.end_effector)
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
        time.sleep(1)

        waypoints = []
        current = self.arm.get_current_pose(self.end_effector).pose

        corners = [
            (0, side_length, 0),
            (side_length, 0, 0),
            (0, -side_length, 0),
            (-side_length, 0, 0),
        ]

        temp = deepcopy(current)
        for dx, dy, dz in corners:
            temp.position.x += dx
            temp.position.y += dy
            temp.position.z += dz
            waypoints.append(deepcopy(temp))
            for _ in range(10):
                interp = deepcopy(temp)
                waypoints.append(interp)

        self.arm.set_start_state_to_current_state()
        plan_result = self.arm.plan_cartesian_path(
            waypoints, 0.005, 0.0, True
        )
        if plan_result and plan_result.fraction >= 1.0:
            self.get_logger().info('方形路径规划成功!')
            self.arm.execute(plan_result.trajectory)
        else:
            fraction = plan_result.fraction if plan_result else 0
            self.get_logger().error(f'方形路径失败: {fraction*100:.1f}%')

        time.sleep(1)
        home = [0.0]*6
        self.arm.set_start_state_to_current_state()
        self.arm.set_joint_value_target(home)
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = SquarePathPlanner()
    node.run_square()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 18.1.5 编写避障规划程序
创建 `ch18_cartesian/obstacle_avoidance.py`（参考实现：`src/lab_code/ch18_lab/moveit_pick_place_lab/obstacles_demo.py`）:
```python
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningComponent
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from shape_msgs.msg import SolidPrimitive
from tf_transformations import quaternion_from_euler
import time
import math

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()
        self.scene_pub = self.create_publisher(PlanningScene, 'planning_scene', 10)
        time.sleep(1)

        self.arm = PlanningComponent(
            self.moveit, 'xarm_group', 'gripper_centor_link'
        )
        self.arm.set_pose_reference_frame('base_link')
        self.arm.set_goal_position_tolerance(0.02)
        self.arm.set_goal_orientation_tolerance(0.03)

    def add_box(self, object_id, x, y, z, size, frame_id='base_link'):
        co = CollisionObject()
        co.id = object_id
        co.header.frame_id = frame_id
        co.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = size
        co.primitives = [primitive]
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        co.primitive_poses = [pose.pose]
        self.planning_scene_monitor.process_collision_object(co)

    def add_cylinder(self, object_id, x, y, z, height, radius, frame_id='base_link'):
        co = CollisionObject()
        co.id = object_id
        co.header.frame_id = frame_id
        co.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [height, radius]
        co.primitives = [primitive]
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        co.primitive_poses = [pose.pose]
        self.planning_scene_monitor.process_collision_object(co)

    def remove_object(self, object_id):
        co = CollisionObject()
        co.id = object_id
        co.operation = CollisionObject.REMOVE
        self.planning_scene_monitor.process_collision_object(co)

    def wait_for_object(self, obstacle_name, timeout=4):
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds / 1e9 < timeout:
            known = self.planning_scene_monitor.get_known_object_names()
            if obstacle_name in known:
                return True
            time.sleep(0.1)
        return False

    def run(self):
        self.get_logger().info('=== 避障规划演示 ===')
        input("按Enter添加桌子...")
        self.add_box('table', 0.0, 0.0, -0.005, [1.0, 1.2, 0.01])
        self.wait_for_object('table')

        input("按Enter添加障碍物1 (圆柱)...")
        self.add_cylinder('obstacle1', 0.3, 0.0, 0.15, 0.3, 0.05)
        self.wait_for_object('obstacle1')

        input("按Enter添加障碍物2 (方块)...")
        self.add_box('obstacle2', 0.3, -0.2, 0.15, [0.15, 0.05, 0.3])
        self.wait_for_object('obstacle2')

        input("按Enter添加障碍物3 (方块)...")
        self.add_box('obstacle3', 0.3, 0.2, 0.15, [0.15, 0.05, 0.3])
        self.wait_for_object('obstacle3')

        input("按Enter改变障碍物颜色...")
        colors = [
            (1.0, 0.0, 0.0, 1.0),
            (0.0, 1.0, 0.0, 1.0),
            (0.0, 0.0, 1.0, 1.0),
        ]
        p = PlanningScene()
        p.is_diff = True
        for i, obj_id in enumerate(['table', 'obstacle1', 'obstacle2']):
            oc = ObjectColor()
            oc.id = obj_id
            oc.color.r, oc.color.g, oc.color.b, oc.color.a = colors[i]
            p.object_colors.append(oc)
        self.scene_pub.publish(p)
        time.sleep(1)

        input("按Enter执行避障规划...")
        self.arm.set_start_state_to_current_state()

        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = 0.25
        target_pose.pose.position.y = -0.4
        target_pose.pose.position.z = 0.25
        q = quaternion_from_euler(0, 0, -math.pi/4)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        self.arm.set_pose_target(target_pose.pose, 'gripper_centor_link')
        plan_result = self.arm.plan()
        if plan_result:
            self.arm.execute(plan_result.trajectory)
            self.get_logger().info('避障规划成功! 路径已避开障碍物')
        else:
            self.get_logger().error('避障规划失败')

        input("按Enter移除所有障碍物...")
        for obj_id in ['table', 'obstacle1', 'obstacle2', 'obstacle3']:
            self.remove_object(obj_id)
        time.sleep(1)

        self.get_logger().info('避障演示完成')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 18.1.6 配置setup.py并编译运行
```python
entry_points={
    'console_scripts': [
        'line_path = ch18_cartesian.line_path:main',
        'square_path = ch18_cartesian.square_path:main',
        'obstacle_avoidance = ch18_cartesian.obstacle_avoidance:main',
    ],
},
```

```bash
cd ~/ros2_arm_ws
colcon build --packages-select ch18_cartesian
source install/setup.bash

# 直线路径
ros2 run ch18_cartesian line_path

# 方形路径
ros2 run ch18_cartesian square_path

# 避障规划 (交互式, 按Enter逐步执行)
ros2 run ch18_cartesian obstacle_avoidance
```

### 18.1.7 对比笛卡尔与非笛卡尔模式
创建 `ch18_cartesian/comparison.py`:
```python
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningComponent
from geometry_msgs.msg import Pose
from copy import deepcopy
import time

class Comparison(Node):
    def __init__(self):
        super().__init__('comparison')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.arm = PlanningComponent(
            self.moveit, 'xarm_group', 'gripper_centor_link'
        )
        self.arm.set_pose_reference_frame('base_link')
        self.end_effector = 'gripper_centor_link'

    def move_via_poses(self, waypoints, cartesian=False):
        if cartesian:
            self.arm.set_start_state_to_current_state()
            plan = self.arm.plan_cartesian_path(waypoints, 0.01, 0.0, True)
            if plan and plan.fraction >= 1.0:
                self.arm.execute(plan.trajectory)
                return
        else:
            for wp in waypoints:
                self.arm.set_start_state_to_current_state()
                self.arm.set_pose_target(wp, self.end_effector)
                plan = self.arm.plan()
                if plan:
                    self.arm.execute(plan.trajectory)
                time.sleep(1)

    def run(self):
        start_pose = Pose()
        start_pose.position.x = 0.3
        start_pose.position.y = 0.0
        start_pose.position.z = 0.4
        start_pose.orientation.w = 1.0

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(start_pose, self.end_effector)
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
        time.sleep(1)

        current = self.arm.get_current_pose(self.end_effector).pose
        waypoints = []
        w = deepcopy(current)
        w.position.x += 0.1; w.position.y += 0.1; w.position.z -= 0.05
        waypoints.append(deepcopy(w))
        w.position.x -= 0.2; w.position.y -= 0.1; w.position.z += 0.1
        waypoints.append(deepcopy(w))
        w.position.x += 0.1
        waypoints.append(deepcopy(w))

        self.get_logger().info('=== 非笛卡尔模式 (逐点运动) ===')
        self.move_via_poses(waypoints, cartesian=False)
        time.sleep(1)

        self.arm.set_start_state_to_current_state()
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
        time.sleep(1)

        self.get_logger().info('=== 笛卡尔模式 (直线路径) ===')
        self.move_via_poses(waypoints, cartesian=True)

def main(args=None):
    rclpy.init(args=args)
    node = Comparison()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 18.2 抓取与放置

### 18.2.1 运行参考代码
启动 xArm 仿真（见前置说明）后，另开终端运行：

```bash
ros2 run moveit_pick_place_lab target_publisher    # 先发布目标
ros2 run moveit_pick_place_lab pick_place_demo     # 抓取-放置全流程
ros2 run moveit_pick_place_lab attach_object_demo  # 物体附着/分离
```

`pick_place_demo` 依次完成接近→抓取→提升→移动→放置→归位，终端输出各阶段规划结果；RViz/Gazebo 中可见目标物体被拾起与放下。

### 18.2.2 创建实验包
```bash
cd ~/ros2_arm_ws/src
ros2 pkg create ch18_pick_place --build-type ament_python --dependencies rclpy moveit
cd ch18_pick_place
mkdir -p ch18_pick_place
```

### 18.2.3 编写基础的Pick-and-Place程序
创建 `ch18_pick_place/pick_place_demo.py`:
```python
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningComponent
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from tf_transformations import quaternion_from_euler
from copy import deepcopy
import time
import math

class PickPlaceDemo(Node):
    ARM_GROUP = 'xarm_group'
    GRIPPER_GROUP = 'gripper_group'
    GRIPPER_FRAME = 'gripper_centor_link'
    GRIPPER_OPEN = [0.65, 0.65]
    GRIPPER_CLOSE = [0.0, 0.0]
    REFERENCE_FRAME = 'base_link'

    def __init__(self):
        super().__init__('pick_place_demo')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.psm = self.moveit.get_planning_scene_monitor()
        self.arm = PlanningComponent(
            self.moveit, self.ARM_GROUP, self.GRIPPER_FRAME
        )
        self.gripper = PlanningComponent(
            self.moveit, self.GRIPPER_GROUP, self.GRIPPER_FRAME
        )
        self.arm.set_pose_reference_frame(self.REFERENCE_FRAME)
        time.sleep(1)
        self.get_logger().info('Pick and Place Demo已启动')

    def add_collision_box(self, object_id, x, y, z, size):
        co = CollisionObject()
        co.id = object_id
        co.header.frame_id = self.REFERENCE_FRAME
        co.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = size
        co.primitives = [primitive]
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        co.primitive_poses = [pose]
        self.psm.process_collision_object(co)

    def remove_object(self, object_id):
        co = CollisionObject()
        co.id = object_id
        co.operation = CollisionObject.REMOVE
        self.psm.process_collision_object(co)

    def attach_object(self, object_id):
        aco = AttachedCollisionObject()
        aco.link_name = self.GRIPPER_FRAME
        aco.object.id = object_id
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = ['gripper_1_link', 'gripper_2_link']
        self.psm.process_attached_collision_object(aco)
        self.get_logger().info(f'附着物体: {object_id}')

    def detach_object(self, object_id):
        aco = AttachedCollisionObject()
        aco.link_name = self.GRIPPER_FRAME
        aco.object.id = object_id
        aco.object.operation = CollisionObject.REMOVE
        self.psm.process_attached_collision_object(aco)
        self.get_logger().info(f'分离物体: {object_id}')

    def move_gripper(self, position):
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_joint_value_target(position)
        plan = self.gripper.plan()
        if plan:
            self.gripper.execute(plan.trajectory)

    def move_arm_to_joints(self, joints):
        self.arm.set_start_state_to_current_state()
        self.arm.set_joint_value_target(joints)
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
            return True
        return False

    def move_arm_to_pose(self, target_pose):
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.GRIPPER_FRAME)
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
            return True
        return False

    def run_pick_and_place(self):
        self.get_logger().info('=== Pick and Place 演示 ===')

        table_id = 'table'
        target_id = 'target_cube'

        self.get_logger().info('添加桌子和目标物体到规划场景')
        self.add_collision_box(table_id, 0.0, 0.0, -0.005, [1.0, 1.2, 0.01])
        time.sleep(0.5)

        cube_size = [0.04, 0.04, 0.04]
        cube_x, cube_y, cube_z = 0.35, 0.0, 0.04
        self.add_collision_box(target_id, cube_x, cube_y, cube_z, cube_size)
        time.sleep(0.5)

        home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pre_grasp = [0.3, -0.5, 0.5, -0.2, 0.0, 0.0]
        grasp = [0.3, -0.2, 0.1, 0.0, 0.0, 0.0]
        lift = [0.3, -0.5, 0.5, -0.2, 0.0, 0.0]
        pre_place = [-0.3, -0.5, 0.5, -0.2, 0.0, 0.0]
        place = [-0.3, -0.2, 0.1, 0.0, 0.0, 0.0]

        self.move_arm_to_joints(home)
        time.sleep(1)

        self.get_logger().info('[1] 移动到手爪预张开位置')
        self.move_gripper(self.GRIPPER_OPEN)
        time.sleep(0.5)

        self.get_logger().info('[2] 移动到预抓取位姿')
        self.move_arm_to_joints(pre_grasp)
        time.sleep(1)

        self.get_logger().info('[3] 抓取物体')
        self.remove_object(target_id)
        self.move_arm_to_joints(grasp)
        time.sleep(1)

        self.get_logger().info('[4] 闭合手爪')
        self.move_gripper(self.GRIPPER_CLOSE)
        time.sleep(0.5)
        self.attach_object(target_id)
        time.sleep(0.5)

        self.get_logger().info('[5] 提起物体')
        self.move_arm_to_joints(lift)
        time.sleep(1)

        self.get_logger().info('[6] 移动到放置区域')
        self.move_arm_to_joints(pre_place)
        time.sleep(1)

        self.get_logger().info('[7] 放置物体')
        self.move_arm_to_joints(place)
        time.sleep(1)

        self.get_logger().info('[8] 张开手爪释放物体')
        self.detach_object(target_id)
        place_x, place_y, place_z = -0.3, -0.2, 0.04
        self.add_collision_box(
            target_id + '_placed', place_x, place_y, place_z, cube_size
        )
        time.sleep(0.5)
        self.move_gripper(self.GRIPPER_OPEN)
        time.sleep(1)

        self.get_logger().info('[9] 回到Home')
        self.move_arm_to_joints(home)

        self.get_logger().info('清理场景')
        self.remove_object(table_id)
        self.remove_object(target_id + '_placed')

        self.get_logger().info('Pick and Place 完成!')

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceDemo()
    node.run_pick_and_place()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 18.2.4 编写带笛卡尔路径的抓取程序
创建 `ch18_pick_place/cartesian_pick_place.py`:
```python
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningComponent
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from copy import deepcopy
import time
import numpy as np

class CartesianPickPlace(Node):
    def __init__(self):
        super().__init__('cartesian_pick_place')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.psm = self.moveit.get_planning_scene_monitor()
        self.arm = PlanningComponent(
            self.moveit, 'xarm_group', 'gripper_centor_link'
        )
        self.gripper = PlanningComponent(
            self.moveit, 'gripper_group', 'gripper_centor_link'
        )
        self.arm.set_pose_reference_frame('base_link')

    def add_box(self, obj_id, x, y, z, size):
        co = CollisionObject()
        co.id = obj_id
        co.header.frame_id = 'base_link'
        co.operation = CollisionObject.ADD
        p = SolidPrimitive()
        p.type = SolidPrimitive.BOX
        p.dimensions = size
        co.primitives = [p]
        pose = Pose()
        pose.position.x = x; pose.position.y = y; pose.position.z = z
        pose.orientation.w = 1.0
        co.primitive_poses = [pose]
        self.psm.process_collision_object(co)

    def remove_obj(self, obj_id):
        co = CollisionObject()
        co.id = obj_id
        co.operation = CollisionObject.REMOVE
        self.psm.process_collision_object(co)

    def attach(self, obj_id):
        aco = AttachedCollisionObject()
        aco.link_name = 'gripper_centor_link'
        aco.object.id = obj_id
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = ['gripper_1_link', 'gripper_2_link']
        self.psm.process_attached_collision_object(aco)

    def detach(self, obj_id):
        aco = AttachedCollisionObject()
        aco.link_name = 'gripper_centor_link'
        aco.object.id = obj_id
        aco.object.operation = CollisionObject.REMOVE
        self.psm.process_attached_collision_object(aco)

    def run_cartesian_pick_place(self):
        self.get_logger().info('=== 笛卡尔路径 Pick and Place ===')

        target_id = 'target'
        table_id = 'table'

        self.add_box(table_id, 0.0, 0.0, -0.005, [1.0, 1.2, 0.01])
        self.add_box(target_id, 0.35, 0.0, 0.04, [0.04, 0.04, 0.04])

        self.gripper.set_start_state_to_current_state()
        self.gripper.set_joint_value_target([0.65, 0.65])
        plan = self.gripper.plan()
        if plan: self.gripper.execute(plan.trajectory)
        time.sleep(0.5)

        self.arm.set_start_state_to_current_state()
        home_pose = Pose()
        home_pose.position.x = 0.3
        home_pose.position.y = 0.0
        home_pose.position.z = 0.4
        home_pose.orientation.w = 1.0
        self.arm.set_pose_target(home_pose, 'gripper_centor_link')
        plan = self.arm.plan()
        if plan: self.arm.execute(plan.trajectory)
        time.sleep(0.5)

        self.remove_obj(target_id)

        self.get_logger().info('笛卡尔下降抓取')
        current = self.arm.get_current_pose('gripper_centor_link').pose
        waypoints = []
        for i in range(20):
            wp = deepcopy(current)
            wp.position.z -= 0.01 * i
            waypoints.append(wp)

        self.arm.set_start_state_to_current_state()
        result = self.arm.plan_cartesian_path(waypoints, 0.005, 0.0, True)
        if result and result.fraction >= 1.0:
            self.arm.execute(result.trajectory)
        time.sleep(0.5)

        self.gripper.set_start_state_to_current_state()
        self.gripper.set_joint_value_target([0.0, 0.0])
        plan = self.gripper.plan()
        if plan: self.gripper.execute(plan.trajectory)
        self.attach(target_id)
        time.sleep(0.5)

        self.get_logger().info('笛卡尔提升')
        current = self.arm.get_current_pose('gripper_centor_link').pose
        lift_waypoints = []
        for i in range(15):
            wp = deepcopy(current)
            wp.position.z += 0.01 * i
            lift_waypoints.append(wp)

        self.arm.set_start_state_to_current_state()
        result = self.arm.plan_cartesian_path(lift_waypoints, 0.005, 0.0, True)
        if result and result.fraction >= 1.0:
            self.arm.execute(result.trajectory)
        time.sleep(0.5)

        self.get_logger().info('移动到放置区域')
        place_pose = Pose()
        place_pose.position.x = 0.2
        place_pose.position.y = -0.25
        place_pose.position.z = 0.3
        place_pose.orientation.w = 1.0

        self.arm.set_start_state_to_current_state()
        plan = self.arm.plan()
        self.arm.set_pose_target(place_pose, 'gripper_centor_link')
        plan = self.arm.plan()
        if plan: self.arm.execute(plan.trajectory)
        time.sleep(0.5)

        self.get_logger().info('笛卡尔下降放置')
        current = self.arm.get_current_pose('gripper_centor_link').pose
        place_down = []
        for i in range(18):
            wp = deepcopy(current)
            wp.position.z -= 0.01 * i
            place_down.append(wp)

        self.arm.set_start_state_to_current_state()
        result = self.arm.plan_cartesian_path(place_down, 0.005, 0.0, True)
        if result and result.fraction >= 1.0:
            self.arm.execute(result.trajectory)
        time.sleep(0.5)

        self.detach(target_id)
        self.add_box(target_id + '_placed', 0.2, -0.25, 0.04, [0.04]*3)
        time.sleep(0.5)

        self.gripper.set_start_state_to_current_state()
        self.gripper.set_joint_value_target([0.65, 0.65])
        plan = self.gripper.plan()
        if plan: self.gripper.execute(plan.trajectory)
        time.sleep(0.5)

        self.arm.set_start_state_to_current_state()
        home_joints = [0.0]*6
        self.arm.set_joint_value_target(home_joints)
        plan = self.arm.plan()
        if plan: self.arm.execute(plan.trajectory)

        self.remove_obj(table_id)
        self.remove_obj(target_id + '_placed')
        self.get_logger().info('笛卡尔路径 Pick and Place 完成!')

def main(args=None):
    rclpy.init(args=args)
    node = CartesianPickPlace()
    node.run_cartesian_pick_place()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 18.2.5 编写抓取放置服务端
创建 `ch18_pick_place/pick_place_server.py`:
```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from moveit.planning import MoveItPy, PlanningComponent
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
import time

class PickPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_place_server')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.psm = self.moveit.get_planning_scene_monitor()
        self.arm = PlanningComponent(
            self.moveit, 'xarm_group', 'gripper_centor_link'
        )
        self.gripper = PlanningComponent(
            self.moveit, 'gripper_group', 'gripper_centor_link'
        )
        self.srv = self.create_service(Trigger, '/pick_place', self.callback)
        self.get_logger().info('Pick Place Server ready. Call /pick_place')

    def callback(self, request, response):
        self.get_logger().info('触发Pick and Place')
        try:
            self.execute_pick_place()
            response.success = True
            response.message = 'Pick and Place succeeded'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def execute_pick_place(self):
        home = [0.0]*6
        pre = [0.3, -0.5, 0.5, -0.2, 0.0, 0.0]
        pick = [0.3, -0.2, 0.1, 0.0, 0.0, 0.0]
        place = [-0.3, -0.2, 0.1, 0.0, 0.0, 0.0]

        self.arm.set_start_state_to_current_state()
        self.arm.set_joint_value_target(home)
        plan = self.arm.plan()
        if plan: self.arm.execute(plan.trajectory)
        time.sleep(0.5)

        self.gripper.set_start_state_to_current_state()
        self.gripper.set_joint_value_target([0.65]*2)
        plan = self.gripper.plan()
        if plan: self.gripper.execute(plan.trajectory)
        time.sleep(0.5)

        for target in [pre, pick, place, home]:
            self.arm.set_start_state_to_current_state()
            self.arm.set_joint_value_target(target)
            plan = self.arm.plan()
            if plan: self.arm.execute(plan.trajectory)
            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 18.2.6 配置setup.py并编译运行
```python
entry_points={
    'console_scripts': [
        'pick_place_demo = ch18_pick_place.pick_place_demo:main',
        'cartesian_pick_place = ch18_pick_place.cartesian_pick_place:main',
        'pick_place_server = ch18_pick_place.pick_place_server:main',
    ],
},
```

```bash
cd ~/ros2_arm_ws
colcon build --packages-select ch18_pick_place
source install/setup.bash

# 在MoveIt2演示环境运行的同时
ros2 run ch18_pick_place pick_place_demo
# 或
ros2 run ch18_pick_place cartesian_pick_place
# 或通过服务触发
ros2 run ch18_pick_place pick_place_server
ros2 service call /pick_place std_srvs/srv/Trigger
```

### 18.2.7 观察附着的效果
在RViz2中:
- 抓取前: 场景中桌面上有一个绿色方块
- 抓取时: 方块从场景中消失(被remove)
- 闭合手爪后: 方块附着到gripper_centor_link, 随机械臂运动
- 放置后: 方块重新出现在放置位置, 从末端分离

### 18.2.8 修改物体尺寸和位置
修改代码中的立方体尺寸和位置参数, 重新运行:
```python
# 修改为目标物体在不同位置
cube_x, cube_y, cube_z = 0.4, 0.1, 0.04
# 或不同尺寸
cube_size = [0.06, 0.06, 0.06]
```

## 实验结果与分析
- 笛卡尔路径规划使得机械臂末端沿直线轨迹运动, 轨迹在RViz2中显示为直线
- 非笛卡尔模式下, 各路径点之间是关节空间的插值, 末端走曲线而非直线
- 避障规划中, 添加障碍物后MoveIt2自动生成绕行路径
- eef_step参数影响笛卡尔路径规划的精度和成功率
- AttachedCollisionObject使得物体能跟随末端执行器移动
- 抓取流程: 预抓取→下降→闭合→attach→提升→移动→下降→detach→提升
- 完整的Pick-and-Place需要协调手臂和手爪的配合运动
- 规划场景中的物体管理对碰撞检测至关重要
- 参考代码 `moveit_pick_place_lab` 的 `beeline_demo`/`obstacles_demo`/`pick_place_demo`/`attach_object_demo` 与本章自实现程序一一对应，可用于对照验证

## 思考题
1. plan_cartesian_path中的eef_step参数如何影响路径规划?
2. 在障碍物密集的环境中, 如何提高避障规划的成功率?
3. 笛卡尔路径规划失败时, 如何通过降采样或分段规划来解决问题?
4. 怎样在RViz2中实时显示末端执行器的运动轨迹?
5. attach和detach操作对碰撞检测有什么影响?
6. 如果抓取前后物体的位姿没有正确更新, 会有什么问题?
7. 如何实现多物体的连续抓取和放置?
8. 实际机器人抓取中, 如何通过力反馈确定抓取是否成功?
