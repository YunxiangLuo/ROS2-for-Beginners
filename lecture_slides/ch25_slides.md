# 第25章 ROS2机械臂建模

---

## 学习目标
- 掌握URDF语法规则
- 学会创建完整的机械臂模型
- 理解Xacro的高级特性
- 能够在RViz2中可视化自定义模型

---

## URDF概述
- Unified Robot Description Format
- XML格式描述机器人模型
- 组成: link(连杆) + joint(关节)
- 每个link有visual/collision/inertial

---

## link定义
- visual: 可视化几何体
  - geometry: box/cylinder/sphere/mesh
  - origin: 相对坐标
  - material: 颜色/纹理
- collision: 碰撞检测几何体
- inertial: 物理属性(质量+惯性矩)

---

## joint类型
- revolute: 旋转关节 (常用)
- prismatic: 平移关节
- fixed: 固定关节
- continuous: 连续旋转关节
- floating: 浮动关节 (6DOF)
- planar: 平面关节 (3DOF)

---

## joint属性
- parent/child: 连杆连接关系
- origin: 关节原点坐标
- axis: 旋转/平移轴方向
- limit: 运动范围
  - lower/upper: 角度/位移限位
  - effort: 最大力矩
  - velocity: 最大速度

---

## 3自由度机械臂示例
- base_link → joint1(腰部, Z轴旋转) → link1
- link1 → joint2(肩部, Y轴旋转) → link2
- link2 → joint3(肘部, Y轴旋转) → link3
- 每个link指定几何形状和颜色

---

## URDF检查工具
- check_urdf: 语法和拓扑检查
- urdf_to_graphiz: 生成结构图PDF
- 验证:
  - 是否为树形结构
  - 是否只有一个root link
  - joint参数是否完整

---

## 为何需要Xacro?
- URDF的局限性:
  - 大量重复代码
  - 不支持变量和计算
  - 难以模块化
- Xacro: XML Macros
  - 常量定义
  - 数学表达式
  - 宏定义和调用
  - 文件包含

---

## Xacro常量
```xml
<xacro:property name="PI" value="3.14159"/>
<xacro:property name="arm_length" value="0.3"/>
```
- 使用: ${PI}, ${arm_length}
- 一处修改, 全局生效

---

## Xacro数学运算
- 支持: +, -, *, /, 三角函数
- 示例:
  - ${M_PI/2}
  - ${arm_length * 2}
  - ${mass * gravity}
- 自动替换为计算结果

---

## Xacro宏定义
```xml
<xacro:macro name="box_inertial" params="m w h d">
    <inertial>
        <mass value="${m}"/>
        <inertia .../>
    </inertial>
</xacro:macro>
```
- 调用: <box_inertial m="1" w="0.1" h="0.2" d="0.15"/>

---

## Xacro文件包含
```xml
<xacro:include filename="$(find pkg)/urdf/materials.xacro"/>
<xacro:include filename="$(find pkg)/urdf/macros.xacro"/>
```
- 模块化管理
- 复用公共定义
- 便于团队协作

---

## 夹爪建模
- 固定关节: gripper_base
- 棱柱关节: finger1_joint (Y轴移动)
- 棱柱关节: finger2_joint (Y轴反向移动)
- 开合范围: 通过upper/lower控制

---

## robot_state_publisher
- 订阅 /joint_states
- 读取URDF/Xacro模型
- 发布TF变换
- Command函数: 动态加载模型

---

## 模型调试
- RViz2显示常见问题:
  - Red模型: 缺少robot_description
  - 关节不运动: 无joint_states话题
  - 位置不对: origin坐标错误
  - 颜色不对: material定义错误

---

## Xacro转换URDF
```bash
# 预处理Xacro生成URDF
xacro model.xacro > model.urdf

# 检查语法
check_urdf model.urdf
```

---

## 模型优化建议
- 简化collision几何体(提高计算效率)
- 正确设置inertial参数(仿真用)
- 合理选择joint limit(匹配真实机器人)
- 使用mesh文件替代基本几何体(更真实)

---

## 思考
- 为什么collision和visual可以不同?
- 如何实现多指手爪?
- URDF支持哪些几何体格式?
- 如何从CAD模型导入URDF?

---

## 总结
- URDF是ROS机器人建模的标准格式
- link + joint 定义完整的运动链
- Xacro通过宏和变量简化模型描述
- robot_state_publisher + URDF = 完整TF树
- RViz2提供3D可视化验证
