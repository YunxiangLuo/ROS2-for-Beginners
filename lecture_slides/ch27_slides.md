# 第27章 MoveIt2 Python规划

---

## 学习目标
- 掌握MoveItPy的编程接口
- 学会关节空间和笛卡尔空间规划
- 实现速度控制和手爪控制
- 编写完整的运动序列程序

---

## MoveItPy概述
- MoveIt2的Python API
- 基于moveit.planning模块
- 核心类: MoveItPy, PlanningComponent
- 支持: FK, IK, 笛卡尔规划

---

## 初始化MoveItPy
```python
from moveit.planning import MoveItPy, PlanningComponent

moveit = MoveItPy(node_name='moveit_py')
arm = PlanningComponent(
    moveit, 'xarm_group', 'gripper_centor_link'
)
```
- node_name: 内部节点名称
- 第二个参数: 规划组名称
- 第三个参数: 末端执行器名称

---

## 关节空间规划
```python
arm.set_start_state_to_current_state()
arm.set_joint_value_target([0.5, -0.3, ...])
plan_result = arm.plan()
arm.execute(plan_result.trajectory)
```
- set_joint_value_target: 设置关节目标
- plan(): 调用规划器
- execute(): 执行轨迹

---

## 逆运动学规划
```python
target_pose = PoseStamped()
target_pose.pose.position.x = 0.3
target_pose.pose.orientation.w = 1.0

arm.set_start_state_to_current_state()
arm.set_pose_target(target_pose.pose, end_effector)
plan_result = arm.plan()
```
- set_pose_target: 设置末端位姿
- 自动调用IK求解器
- 可设置位置/姿态容差

---

## 设置规划参数
```python
arm.set_goal_position_tolerance(0.01)
arm.set_goal_orientation_tolerance(0.02)
arm.set_max_velocity_scaling_factor(0.5)
arm.set_max_acceleration_scaling_factor(0.5)
```
- position_tolerance: 位置容差(m)
- orientation_tolerance: 姿态容差(rad)
- velocity_scaling: 速度缩放(0~1)
- acceleration_scaling: 加速度缩放

---

## 预设位姿
```python
arm.set_named_target('home')
plan = arm.plan()
if plan:
    arm.execute(plan.trajectory)
```
- set_named_target: 使用预设位姿
- 在Setup Assistant中定义
- 常用: home, vertical, horizontal

---

## 规划结果处理
- plan_result.trajectory: RobotTrajectory
- plan_result.fraction: 笛卡尔路径完成度
- plan_result.error_code: 错误码
- 总是检查plan_result是否为None

---

## 速度控制
- set_max_velocity_scaling_factor(factor)
- factor=0.1: 最慢
- factor=1.0: 最快
- 加速度也类似
- 用于调试和演示

---

## 手爪控制
```python
gripper = PlanningComponent(moveit, 'gripper_group', ...)
gripper.set_joint_value_target([0.65, 0.65])
```
- 手爪也是规划组
- 棱柱关节设置位移值
- 旋转关节设置角度值

---

## 获取当前状态
```python
state = moveit.get_robot_state()
state.get_joint_positions('arm_1_joint')
state.get_pose('gripper_centor_link')
```
- 实时获取关节角度
- 实时获取末端位姿
- 用于状态监控

---

## 多步骤运动序列
1. Home → 准备位姿
2. 张开手爪
3. 下降到抓取位姿
4. 闭合手爪
5. 提起物体
6. 移动到放置位姿
7. 张开手爪释放
8. 回到Home

---

## 序列控制要点
- 每步之间加time.sleep()等待
- 检查每步的规划结果
- 设置合理的速度缩放
- 注意手爪开合与手臂运动配合

---

## 错误处理
```python
if plan_result:
    arm.execute(plan_result.trajectory)
else:
    self.get_logger().warn('规划失败')
```
- 规划失败: 调整目标或容差
- IK无解: 改变目标位姿
- 碰撞: 添加障碍物到场景
- 超时: 增加规划时间

---

## 速度对比演示
- 低速(0.2): 平滑但慢
- 中速(0.5): 平衡
- 高速(1.0): 快速但有冲击
- 根据任务选择合适的参数

---

## 规划器选择
```python
plan = arm.plan(
    target=joints,
    planner_id='RRTConnectkConfigDefault',
    planning_time=5.0,
)
```
- planner_id: 规划器名称
- planning_time: 最大规划时间(秒)
- 不同规划器有不同的路径特性

---

## 思考
- plan.getJointTrajectory()返回什么?
- 如何获取轨迹的执行时间?
- 多个规划组如何协同?
- 如何实现平滑的轨迹过渡?

---

## 总结
- MoveItPy提供完整的Python控制接口
- 关节空间规划简单直接
- IK规划更加灵活
- 速度/加速度控制精细调节
- 多步骤序列可完成复杂任务
