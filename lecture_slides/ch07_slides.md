# 第7章 PPT：TF2 坐标变换系统

> 共 14 页

---

## P1 · 标题页
**TF2 坐标变换系统** | 第7章 | 2课时

## P2 · 学习目标
- 理解坐标系树 (Frame Tree) 设计原理
- 掌握静态变换与动态变换的广播
- 使用 Buffer + lookup_transform 查询变换
- 实现坐标点变换与时间同步
- 熟练使用 TF2 调试工具

## P3 · 坐标系树结构

```
       map
        │
       odom
        │
  base_footprint
        │
    base_link ──────────────────┐
   ┌────┼────┬────────┐         │
laser  camera imu  left_wheel  right_wheel
```

**规则**：有向无环图 (DAG)，每子系只有一个父系

## P4 · 静态变换广播器

程序 7-1：
```python
broadcaster = StaticTransformBroadcaster(self)
t = TransformStamped()
t.header.frame_id = 'base_link'
t.child_frame_id = 'laser_frame'
t.transform.translation.x = 0.2
broadcaster.sendTransform(t)  # 只发一次
```

适用于：传感器固定安装位置、机械臂基座等

## P5 · 动态变换广播器

程序 7-2：
```python
broadcaster = TransformBroadcaster(self)
timer = self.create_timer(0.02, self.publish_tf)
# 每帧更新 odom→base_link 变换
t.header.stamp = self.get_clock().now().to_msg()
broadcaster.sendTransform(t)
```

适用于：里程计、移动机器人位姿 (≥50Hz)

## P6 · 变换监听与查询

程序 7-3：
```python
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, self)
t = tf_buffer.lookup_transform(
    'base_link', 'laser_frame', rclpy.time.Time())
```

- `target`: 目标坐标系
- `source`: 源坐标系
- `time`: 时间戳 (Time() = 最新)

## P7 · 坐标点变换

```python
from tf2_geometry_msgs import do_transform_point

point = PointStamped()
point.header.frame_id = 'laser_frame'
point.point.x = 1.0

t = tf_buffer.lookup_transform('base_link', 'laser_frame', Time())
result = do_transform_point(point, t)
```

## P8 · 时间同步

程序 7-4：
```python
if tf_buffer.can_transform(
    'base_link', 'laser_frame', Time(),
    timeout=Duration(seconds=2.0)):
    t = tf_buffer.lookup_transform(...)
```

- 先等待数据到达，再查询
- 避免启动时序差异导致失败

## P9 · TF 插值机制

| 查询方式 | 说明 |
|---------|------|
| `Time()` | 获取最新缓存变换 |
| `Time(seconds=t)` | 获取历史时间戳变换 (线性插值) |
| 缓存长度 | 默认 10 秒，超时 `LookupException` |

## P10 · 调试工具

```bash
# 实时回显两坐标系变换
ros2 run tf2_ros tf2_echo base_link laser_frame

# 监视所有 TF 广播频率和延迟
ros2 run tf2_ros tf2_monitor

# 生成坐标系树 PDF
ros2 run tf2_tools view_frames
```

## P11 · 广播/监听典型架构

```
┌─────────────┐     /tf      ┌─────────────┐
│ Broadcaster │ ──────────→  │   tf2_ros    │
│  (10-100Hz) │              │   buffer     │
└─────────────┘              └──────┬───────┘
                                    │ lookup_transform()
                              ┌─────▼──────┐
                              │  Listener   │
                              │  (按需查询)  │
                              └────────────┘
```

## P12 · 本章要点

1. TF 坐标系树 = 有向无环图，父→子单向依赖
2. `StaticTransformBroadcaster` 固定变换 vs `TransformBroadcaster` 动态变换
3. `Buffer.lookup_transform(target, source, time)` 查询变换
4. `can_transform` + timeout 实现安全时间同步
5. 点/向量变换: `do_transform_point` / `do_transform_vector`
6. `tf2_echo` / `tf2_monitor` / `view_frames` 三大调试利器

## P13 · 练习题

1. 静态 TF 广播 laser_frame
2. 动态 TF 广播 odom→base_link 圆形轨迹
3. TF 监听器查询 laser→base 位姿
4. 坐标点变换 (laser→base)
5. can_transform 安全超时查询
6. view_frames 生成完整 TF 树 PDF

## P14 · 下章预告
**第8章：URDF/Xacro 机器人建模** — 用 XML 描述机器人结构，RViz2 可视化