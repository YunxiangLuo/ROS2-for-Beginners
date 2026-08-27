# 第28章 笛卡尔与避障

---

## 学习目标
- 掌握笛卡尔空间路径规划方法
- 理解直线路径和逐点运动的区别
- 学会在规划场景中添加障碍物
- 掌握避障规划的原理和技巧

---

## 笛卡尔路径规划
- 末端执行器沿直线/曲线运动
- 中间点通过插值生成
- 适用于: 焊接, 涂胶, 搬运
- 需要高精度轨迹的任务

---

## plan_cartesian_path
```python
plan_result = arm.plan_cartesian_path(
    waypoints,          # 路径点列表
    step=0.01,          # 插值步长(m)
    jump_threshold=0.0, # 跳变阈值
    avoid_collisions=True
)
```
- waypoints: Pose列表
- step: 笛卡尔插值精度
- jump_threshold: 关节跳变检测

---

## 路径完成度(fraction)
- 0~1之间的浮点数
- 1.0: 完整路径规划成功
- < 1.0: 部分完成
- 失败原因: 碰撞/奇异/超限
- 可通过多次尝试提高完成度

---

## 直线 vs 圆弧路径
- 直线: 简单插值, 路径点少
- 圆弧: 需要密集插值, 路径点多
- 参数方程:
  - x = cx + r*cos(theta)
  - y = cy + r*sin(theta)
  - theta从0到2*PI

---

## 笛卡尔 vs 关节空间
| 特性 | 笛卡尔空间 | 关节空间 |
|------|-----------|---------|
| 末端轨迹 | 可控(直线) | 不可控 |
| IK需求 | 需要 | 不需要 |
| 计算量 | 大 | 小 |
| 适用场景 | 精密作业 | 点对点移动 |
| 奇异问题 | 容易出现 | 不容易 |

---

## eef_step参数
- 末端单位位移之间的插值步长
- 值越小: 精度越高, 计算量越大
- 值越大: 精度越低, 可能跳过障碍
- 推荐: 0.005 ~ 0.02

---

## 碰撞检测原理
- FCL库: 快速碰撞检测
- 离散: 检查离散路径点
- 连续: 检查点之间的运动
- 自碰撞: 机械臂自身
- 环境碰撞: 与障碍物

---

## CollisionObject
- BOX: 长方体
- SPHERE: 球体
- CYLINDER: 圆柱体
- MESH: 自定义网格
- 通过PlanningSceneMonitor添加

---

## 添加障碍物
```python
co = CollisionObject()
co.id = 'obstacle1'
co.operation = CollisionObject.ADD
primitive = SolidPrimitive()
primitive.type = SolidPrimitive.BOX
primitive.dimensions = [0.2, 0.2, 0.3]
co.primitives = [primitive]
psm.process_collision_object(co)
```

---

## 物体颜色设置
```python
oc = ObjectColor()
oc.id = 'obstacle1'
oc.color.r = 0.8  # RGB
oc.color.g = 0.0
oc.color.b = 0.0
oc.color.a = 1.0  # Alpha
```
- 通过PlanningScene消息发布
- 便于视觉区分不同障碍物

---

## 避障规划策略
- 添加障碍物到规划场景
- 规划器自动生成绕行路径
- 障碍物密集时可能需要:
  - 增加规划时间
  - 降低速度
  - 改变规划器

---

## 避障失败原因
- 障碍物完全阻挡路径
- 障碍物与机械臂干涉
- 规划时间不足
- 规划器参数不合适
- IK无可行解

---

## 对比演示
- 非笛卡尔模式: 各路径点单独IK规划
  - 末端走曲线, 路径不可控
- 笛卡尔模式: 连续插值
  - 末端走直线, 路径可控
- 通过--ros-args -p cartesian:=True/False切换

---

## 障碍物移除
```python
co = CollisionObject()
co.id = 'obstacle1'
co.operation = CollisionObject.REMOVE
psm.process_collision_object(co)
```
- 动态更新规划场景
- 移除后规划恢复正常

---

## 密集路径点规划
- 大量路径点可能超过规划器限制
- 分段规划: 将长路径分段
- 每次规划10-20个路径点
- 逐步执行, 确保稳定性

---

## 思考
- eef_step = 0.001和0.05有何差异?
- 如何规划复杂曲线(如螺旋线)?
- 避障失败时能自动降级吗?
- 如何在运行时动态更新障碍物?

---

## 总结
- 笛卡尔路径确保末端走直线
- plan_cartesian_path支持多路径点
- 碰撞检测和避障是核心安全机制
- CollisionObject动态管理场景
- 对比非笛卡尔模式理解路径差异
