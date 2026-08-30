# 第7章 PPT：TF2 坐标变换系统

> 共 15 页，标注页码 · 图号与教学文档对应

---

## P1 · 标题页

**TF2 坐标变换系统**

- 课程：ROS2 Python 编程
- 章节：第 7 章
- 课时：2 课时

---

## P2 · 本课学习目标

- 理解 TF 坐标系树结构与 DAG（有向无环图）原则
- 掌握 `StaticTransformBroadcaster` 静态变换广播
- 掌握 `TransformBroadcaster` 动态变换广播
- 用 `Buffer.lookup_transform` 查询变换关系
- 用 `do_transform_point` 完成点坐标变换
- 理解时间同步、插值机制与三大调试工具

---

## P3 · 坐标系树设计原理

典型 XBot-U 坐标系树：

```
map → odom → base_footprint → base_link
                                ├── laser_frame
                                ├── camera_frame
                                └── imu_link
```

- TF2 将坐标系组织为树状结构，确保无环单向依赖
- 各坐标系只能有一个父系（DAG），保证变换查找唯一路径
- 官方类比「地图上的寻人」：知道「世界→机器人」「机器人→机械臂」两张图，就能推得世界下机械臂的位姿，即变换叠加

---

## P4 · TF2 核心 API 模块

```python
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
```

| 模块 | 用途 |
|------|------|
| `Buffer` | 缓存变换历史，提供查询接口 |
| `TransformListener` | 订阅 /tf 话题，与 Buffer 同步 |
| `TransformBroadcaster` | 动态变换广播 |
| `StaticTransformBroadcaster` | 静态变换广播（固定安装关系） |

---

## P5 · 静态变换广播器

```python
class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_pub')
        self.broadcaster = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'
        t.transform.translation.x = 0.2
        t.transform.translation.z = 0.1
        t.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(t)
```

程序 7-1：静态变换只发送一次，适用于固定不变的传感器安装位置。

- 命令行等效：`ros2 run tf2_ros static_transform_publisher --x 0.2 --z 0.1 --frame-id base_link --child-frame-id laser_frame`

---

## P6 · 动态变换广播器

```python
class DynamicTFPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_tf_pub')
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_tf)

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = math.sin(self.count * 0.1)
        t.transform.translation.y = math.cos(self.count * 0.1)
        t.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(t)
```

程序 7-2：动态变换持续广播（如 odom→base_link），约 10Hz 定时更新。

- 广播端发布的是「相对变换」（子帧相对父帧），查询端得到的是「绝对变换」（任意两帧推导结果），二者不可混用

---

## P7 · 监听 TF：lookup_transform

```python
class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.lookup_tf)

    def lookup_tf(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'base_link',          # 目标坐标系
                'laser_frame',        # 源坐标系
                now,                  # 时间戳
                timeout=rclpy.duration.Duration(seconds=1.0))
            t = transform.transform.translation
            self.get_logger().info(
                f'Laser→Base: x={t.x:.3f}, y={t.y:.3f}, z={t.z:.3f}')
        except Exception as e:
            self.get_logger().warn(f'TF 查询失败: {e}')
```

程序 7-3：`lookup_transform(target, source, time)` 查询两个坐标系之间的变换关系。

---

## P8 · 坐标点变换

```python
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

# 将激光雷达坐标系下的点转换到 base_link
point_in_laser = PointStamped()
point_in_laser.header.frame_id = 'laser_frame'
point_in_laser.point.x = 1.0

transform = self.tf_buffer.lookup_transform(
    'base_link', 'laser_frame', rclpy.time.Time())
point_in_base = do_transform_point(point_in_laser, transform)
```

- 先查询变换，再用 `do_transform_point` 施加平移与旋转
- Buffer 延迟约等于话题频率的倒数，工程上常查询过去 50–100 ms 的数据

---

## P9 · 时间同步与插值

```python
def lookup_tf_with_wait(self):
    # 等待直到指定变换可用（waitForTransform 已废弃）
    if self.tf_buffer.can_transform(
        'base_link', 'laser_frame', rclpy.time.Time(),
        timeout=rclpy.duration.Duration(seconds=2.0)):

        transform = self.tf_buffer.lookup_transform(
            'base_link', 'laser_frame', rclpy.time.Time())
```

程序 7-4：`can_transform` 等待变换就绪，避免因时序问题导致查询失败。

- Buffer 默认保存约 10 秒变换历史，未精确命中的时刻自动线性插值（绕旋转轴插值）
- 传 `time=Time()` 取最新；传历史时间戳取该时刻变换（需缓存足够）
- 传感器数据查询应使用数据自身时间戳 `header.stamp`
- 排障两大高频原因：帧名拼写、时间戳不连续

---

## P10 · 命令行调试工具

```bash
# 实时查看两坐标系变换
ros2 run tf2_ros tf2_echo base_link laser_frame

# 监视所有 TF 变换（帧频率与延迟）
ros2 run tf2_ros tf2_monitor

# 生成坐标系树 PDF（frames.pdf）
ros2 run tf2_tools view_frames

# 命令行发布静态变换
ros2 run tf2_ros static_transform_publisher \
  --x 0.2 --z 0.1 --yaw 0.0 \
  --frame-id base_link --child-frame-id laser_frame
```

- 树中出现分叉或多根即声明错误，`view_frames` 可直接看出

---

## P11 · 高频报错与根因对照

| 报错信息 | 根因 |
|----------|------|
| `Could not find a connection between X and Y` | 两帧不在同一棵树（多为静态变换未加载） |
| `Lookup would result in an invalid transformation tree` | 父子关系循环 |
| `Data unavailable` | 时间戳超出 Buffer 缓存窗口 |

- 三类症状对应根因：帧树断裂、时间不同步、广播频率过低
- 练习中复现的每种报错都应对应到根因，形成排障直觉

---

## P12 · 仿真结合实例：查询 Wheeltec 传感器坐标系

```bash
# 终端 1：启动仿真和 RViz，不自动巡航
ros2 launch robot_sim_demo gazebo2.launch.py gui:=true rviz:=true drive:=false

# 终端 2：查询 TF
ros2 topic info /tf
ros2 run tf2_ros tf2_echo base_link laser_link
ros2 run tf2_tools view_frames
```

- RViz 的 TF 显示中可看到底盘到激光雷达、相机的层级关系
- `tf2_echo` 持续输出平移和旋转，`view_frames` 生成当前 TF 树文件
- 移动机器人后再查询，可区分固定的 `base_link → laser_link` 安装变换与随运动变化的底盘变换
- 具体 frame 名称以当前模型和 `ros2 topic echo /tf` 输出为准，不强行套用示例名称

---

## P13 · 本章要点

1. TF2 坐标系统呈树状结构（DAG），每个子系只有一个父系
2. `StaticTransformBroadcaster` 发送固定变换，`TransformBroadcaster` 发送动态变换
3. 用 `Buffer.lookup_transform(target, source, time)` 查询变换关系
4. `can_transform` 加 timeout 实现时间同步等待
5. TF2 自动插值支持历史时间戳查询
6. `tf2_echo`、`tf2_monitor`、`view_frames` 是三大调试工具

---

## P14 · 练习题

1. 编写静态 TF 广播器，发布 base_link → laser_frame 的固定变换（x=0.3, z=0.15）
2. 编写动态 TF 广播器，以 10Hz 发布 odom → base_link 的圆形运动轨迹变换
3. 编写 TF 监听器，每秒查询 laser_frame 在 base_link 中的位姿并输出
4. 在监听器中实现点坐标变换：将 laser_frame 下的 (1.0, 0.0, 0.0) 点转换到 base_link 系下
5. 使用 `can_transform` 实现带超时的安全查询，超时后重试最多 3 次
6. 使用 `tf2_echo`、`tf2_monitor`、`view_frames` 调试完整 TF 树，导出 frames.pdf

---

## P15 · 下章预告

**第 8 章：URDF/Xacro 机器人建模**

- URDF 语法基础与 link/joint 元素
- XACRO 宏与参数化建模
- robot_state_publisher 自动发布 TF
- RViz2 RobotModel 可视化验证