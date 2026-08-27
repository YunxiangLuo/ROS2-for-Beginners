# 第7章：TF2 坐标变换系统

> **课程**：ROS2 Python 编程  
> **章节**：第7章  
> **课时**：2 课时（90 分钟）  

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

---

## 7.6 本章小结

1. TF2 坐标系统呈树状结构 (DAG)，每子系只有一个父系
2. `StaticTransformBroadcaster` 发送固定变换，`TransformBroadcaster` 发送动态变换
3. `Buffer.lookup_transform(target, source, time)` 查询变换关系
4. `can_transform` + timeout 实现时间同步等待
5. TF2 自动插值支持历史时间戳查询
6. `tf2_echo / tf2_monitor / view_frames` 三大调试工具

---

## 7.7 练习题

**练习 7.1**：编写静态 TF 广播器，发布 base_link → laser_frame 的固定变换（x=0.3, z=0.15）。

**练习 7.2**：编写动态 TF 广播器，以 10Hz 频率发布 odom → base_link 的圆形运动轨迹变换。

**练习 7.3**：编写 TF 监听器，每秒查询 laser_frame 在 base_link 中的位姿并输出。

**练习 7.4**：在监听器中实现点坐标变换：将 laser_frame 下的 (1.0, 0.0, 0.0) 点转换到 base_link 系下。

**练习 7.5**：使用 `can_transform` 实现带超时的安全查询，超时后重试最多 3 次。

**练习 7.6**：使用 `tf2_echo`, `tf2_monitor`, `view_frames` 调试完整的 TF 树，导出 frames.pdf。