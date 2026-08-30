# 第7章：TF2 坐标变换系统

> **课程**：ROS2 Python 编程  
> **章节**：第7章  
> **课时**：2 课时（90 分钟）  
> **教学方式**：讲授 + 演示  

---

## 7.1 坐标系树 (Frame Tree) 设计原理

### 知识点 7.1.1：TF 坐标系树

```python
# TF2 将坐标系组织为树状结构，确保无环单向依赖
# 典型 XBot-U 坐标系树：
#   map → odom → base_footprint → base_link
#                                     ├── laser_frame
#                                     ├── camera_frame
#                                     └── imu_link
```

TF2 要求坐标系之间形成有向无环图（DAG），各坐标系只能有一个父系，保证变换查找唯一路径。

### 知识点 7.1.2：TF2 核心 API 模块

```python
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
```

### 知识点 7.1.3：官方要点——TF 树概念与坐标变换直觉

官方 Introduction to tf2 教程以小乌龟双机追踪为例，解释了 TF2 的两条核心思想：第一，每个链接（link）都拥有自己的坐标系，任意两个坐标系之间的变换（平移 + 四元数旋转）可随时查询；第二，帧与帧之间存在父子依附关系，整体构成一棵树（TF Tree），保证任意两帧间路径唯一、不会出现死锁。教程通过 `ros2 run tf2_ros tf2_echo world turtle2` 演示了查询命令，还给出了 `tf2_monitor` 查看频率的类型图。

Articulated Robotics 用「地图上的寻人」类比：每个坐标系是一张以自己为中心的地图，知道「世界→机器人」「机器人→机械臂」两张图的相对关系，就能推得世界下机械臂的位姿——这正是变换叠加的直觉。本节开头的 TF 树图示与此完全对应。

---

## 7.2 静态变换与动态变换

### 知识点 7.2.1：静态变换广播器

```python
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

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

### 知识点 7.2.2：动态变换广播器

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

程序 7-2：动态变换持续广播（如 odom→base_link），100Hz 更新。

---

## 7.3 tf2_ros.Buffer 与 lookup_transform()

### 知识点 7.3.1：监听 TF 变换

```python
from tf2_ros import Buffer, TransformListener

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

### 知识点 7.3.2：TF 坐标点变换

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

### 知识点 7.3.3：官方要点——广播器与监听器的编写要点

官方 Python 广播器教程（Writing a tf2 broadcaster）实现对乌龟位姿的发布：先订阅位姿话题，在回调中构造 `TransformStamped`（frame_id 为父帧 `world`，child_frame_id 为 `turtle1`），填入平移与四元数 `Quaternion_from_euler`，再用 `DynamicBroadcaster.sendTransform()` 持续广播。监听器端则用 `Buffer.lookup_transform(target, source, time)` 一次性查询，常用 `TransformListener(buffer, self)` 使 Buffer 与 TF 话题同步。

一个关键设计：广播端发布的是「相对变换」（子帧相对父帧），而查询端得到的是「绝对变换」（任意两帧推导结果）——二者不可混用。官方教程还演示了在同一个节点里同时注册订阅与监听器，提醒读者 Buffer 的延迟约等于话题频率的倒数，查询最新时刻附近的数据容易报「Data not available」，工程上常查询过去 50–100 ms 的数据。

---

## 7.4 时间同步与插值

### 知识点 7.4.1：waitForTransform 与时间同步

```python
def lookup_tf_with_wait(self):
    # 等待直到指定变换可用
    if self.tf_buffer.can_transform(
        'base_link', 'laser_frame', rclpy.time.Time(),
        timeout=rclpy.duration.Duration(seconds=2.0)):
        
        transform = self.tf_buffer.lookup_transform(
            'base_link', 'laser_frame', rclpy.time.Time())

# 或使用 waitForTransform（已废弃，建议用 can_transform 的 timeout）
```

程序 7-4：`can_transform` 等待变换就绪，避免因时序问题导致查询失败。

### 知识点 7.4.2：TF 插值机制

TF2 自动在两帧变换之间线性插值。调用 `lookup_transform` 时指定 time=Time()（最新）或指定历史时间戳获取该时刻的变换（需有足够缓存数据）。

```python
# 获取最新变换
t = self.tf_buffer.lookup_transform('base_link', 'laser', rclpy.time.Time())

# 获取 2 秒前的变换（如果 buffer 缓存足够）
past = rclpy.time.Time(seconds=2.0)
t = self.tf_buffer.lookup_transform('base_link', 'laser', past)
```

### 知识点 7.4.3：官方要点——时间、缓存与插值机制

Using time in tf2 教程揭示了 TF2 与硬件时钟协作的机制：变换带时间戳，Buffer 保存一段时间（默认为 10 秒）的变换历史；查询时若目标时刻没有精确命中，TF 会基于最近的两个变换做线性插值（绕旋转轴插值）。教程中的例子正是本章 7.2.2 节动态坐标变换的场景：随着乌龟游走，`lookup_transform` 的 `time` 参数传 `frame_stamped.header.stamp`，即传感器数据自身的时刻，确保「传感器数据到达时对应的机器人位姿」被查询。

插值的正确性依赖时间连续：若各帧时间戳大幅跳变（如 `use_sim_time` 未与仿真时间对齐，见第 9 章），TF 会退化为报错——帧名、时间戳是 TF2 排障的两大高频原因。

---

## 7.5 TF2 调试工具

### 知识点 7.5.1：命令行工具

```bash
# 实时查看两坐标系变换
ros2 run tf2_ros tf2_echo base_link laser_frame

# 监视所有 TF 变换
ros2 run tf2_ros tf2_monitor

# 生成坐标系树 PDF
ros2 run tf2_tools view_frames
# 输出 frames.pdf 可用 evince/draw.io 打开

# 静态变换发布
ros2 run tf2_ros static_transform_publisher \
  --x 0.2 --z 0.1 --yaw 0.0 \
  --frame-id base_link --child-frame-id laser_frame
```

### 知识点 7.5.2：官方要点——三大调试工具与高频报错

官方 Debugging tf2 教程系统整理了三大调试工具与典型症状：`tf2_echo` 显示两帧实时变换（帧名拼写错误时会提示根因）；`tf2_monitor` 统计各帧发布时间与延迟（模拟时间未同步时频率会异常）；`view_frames` 生成 PDF 查看完整 TF 树——树中出现分叉或多根即声明错误。本章练习 7.6 正是要求综合使用这三个工具排查完整机器人模型。

官方还总结了三种高频报错的信息：`Could not find a connection between X and Y`（两帧不在同一棵树，多为静态变换未加载）；`Lookup would result in an invalid transformation tree`（父子关系循环）；`Data unavailable`（时间戳过早或过晚，超过了 Buffer 的缓存窗口）。这些症状与「帧树断裂」「时间不同步」「广播频率过低」三类根因一一对应，建议读者把练习中复现的每种报错对应到根因上，形成直觉。

---

## 7.6 本章小结

TF2 坐标系统呈树状结构（DAG），每个子系只有一个父系；`StaticTransformBroadcaster` 发送固定变换，`TransformBroadcaster` 发送动态变换；使用 `Buffer.lookup_transform(target, source, time)` 查询变换关系，并用 `can_transform` 加 timeout 实现时间同步等待；TF2 自动插值支持历史时间戳查询；`tf2_echo`、`tf2_monitor`、`view_frames` 是三大调试工具。

---

## 7.7 练习题

**练习 7.1**：编写静态 TF 广播器，发布 base_link → laser_frame 的固定变换（x=0.3, z=0.15）。

**练习 7.2**：编写动态 TF 广播器，以 10Hz 频率发布 odom → base_link 的圆形运动轨迹变换。

**练习 7.3**：编写 TF 监听器，每秒查询 laser_frame 在 base_link 中的位姿并输出。

**练习 7.4**：在监听器中实现点坐标变换：将 laser_frame 下的 (1.0, 0.0, 0.0) 点转换到 base_link 系下。

**练习 7.5**：使用 `can_transform` 实现带超时的安全查询，超时后重试最多 3 次。

**练习 7.6**：使用 `tf2_echo`, `tf2_monitor`, `view_frames` 调试完整的 TF 树，导出 frames.pdf。

---

## 仿真结合实例（当前仓库）：查询 Wheeltec 传感器坐标系

### 目标与知识点对应

本实例把 `robot_sim_demo` 发布的机器人状态和传感器 TF 接入 TF2 工具链，验证 `base_link`、`laser_link`、`camera_link` 之间的坐标树，以及 `lookup_transform`/`tf2_echo` 的查询方式。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 终端 1：启动仿真和 RViz，不自动巡航
ros2 launch robot_sim_demo gazebo2.launch.py gui:=true rviz:=true drive:=false
```

另开终端执行：

```bash
source install/setup.bash
ros2 topic info /tf
ros2 run tf2_ros tf2_echo base_link laser_link
ros2 run tf2_tools view_frames
```

### 观察结果

在 RViz 的 TF 显示中可以看到机器人底盘到激光雷达、相机的层级关系；`tf2_echo` 输出平移和旋转，持续查询最新变换，`view_frames` 则可生成当前 TF 树文件。将机器人移动后再次查询，可以区分固定的 `base_link → laser_link` 安装变换和随运动变化的底盘相关变换。

### 源码与边界

TF 与状态发布配置位于 `src/robot_sim_demo/config/gazebo2_bridge.yaml`，机器人模型为 `src/robot_sim_demo/models/wheeltec_robot/model.sdf`，RViz 配置为 `src/robot_sim_demo/rviz/museum.rviz`。具体 frame 名称以当前模型和 `ros2 topic echo /tf` 的输出为准；本实例不把实验示例中的 `laser_frame` 名称强行套用到 Wheeltec 模型。

![ch07 TF2 运行输出](../lab_manuals/images/runtime/ch07_tf.gif)


---

> 参考来源：
> - ROS 2 Documentation (Humble) —— Introduction to tf2：https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html
> - ROS 2 Documentation (Humble) —— Writing a tf2 broadcaster (Python)：https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html
> - ROS 2 Documentation (Humble) —— Using time in tf2：https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Using-Time-In-Tf2.html
> - ROS 2 Documentation (Humble) —— Debugging tf2 (tf2_echo, tf2_monitor, view_frames)：https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Debugging-Tf2-With-Tf2-Echo.html
> - ROS 2 Documentation (Humble) —— TF2 教程总览：https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
> - The Construct —— ROS 2 Basics in 5 Days：https://www.theconstructsim.com/
> - Articulated Robotics —— ROS 2 Basics 系列视频：https://www.youtube.com/@ArticulatedRobotics
