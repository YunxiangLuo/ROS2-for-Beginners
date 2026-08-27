# 第3章 PPT：话题通信（Topics）

> 共 16 页，标注页码

---

## P1 · 标题页

**话题通信（Topics）**

- 课程：ROS2 Python 编程
- 章节：第 3 章
- 课时：2 课时

---

## P2 · 本课学习目标

- 掌握发布-订阅模型的工作机制
- 熟悉常用标准消息类型
- 创建自定义消息接口
- 理解 QoS 策略配置
- 掌握多线程执行器与回调组

---

## P3 · 发布-订阅模型

```
Publisher 1 ────┐
Publisher 2 ────┤  Topic: "/chatter"  ──── Subscriber A
Publisher 3 ────┘                          Subscriber B

   异步 · 多对多 · 解耦
```

图 3-1：话题通信模型

---

## P4 · Publisher 关键 API

```python
# 创建发布者
self.pub = self.create_publisher(
    String,          # 消息类型
    'chatter',       # 话题名称
    10)              # QoS 深度

# 发布消息
msg = String()
msg.data = 'Hello'
self.pub.publish(msg)
```

---

## P5 · Publisher 完整代码

程序 3-1：Talker 节点

```python
class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.callback)
        self.count = 0

    def callback(self):
        msg = String()
        msg.data = f'Hello {self.count}'
        self.pub.publish(msg)
        self.count += 1
```

---

## P6 · Subscriber 关键 API

```python
# 创建订阅者
self.sub = self.create_subscription(
    String,              # 消息类型
    'chatter',           # 话题名称
    self.callback,       # 回调函数
    10)                  # QoS 深度

# 回调函数签名
def callback(self, msg):
    print(msg.data)
```

---

## P7 · Subscriber 完整代码

程序 3-2：Listener 节点

```python
class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(
            String, 'chatter', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'收到: {msg.data}')
```

---

## P8 · 常用标准消息类型

```
std_msgs/    → String, Int32, Float64, Bool, Header
sensor_msgs/ → Image, LaserScan, PointCloud2, Imu
geometry_msgs/ → Twist, Pose, Quaternion
```

查看消息定义：
```bash
ros2 interface show std_msgs/msg/String
ros2 interface show geometry_msgs/msg/Twist
```

---

## P9 · 创建自定义消息

```
interfaces_pkg/
├── CMakeLists.txt
├── package.xml
└── msg/
    └── SensorData.msg
```

```python
# SensorData.msg 定义
float64 temperature
float64 humidity
float64 pressure
string device_id
```

---

## P10 · 使用自定义消息

```python
# 在 package.xml 添加
<exec_depend>custom_interfaces</exec_depend>

# Python 导入
from custom_interfaces.msg import SensorData
msg = SensorData()
msg.temperature = 25.5
```

---

## P11 · QoS 兼容性矩阵

```
Publisher ↓  Subscriber → RELIABLE │ BEST_EFFORT
────────────────────────────────────────────
RELIABLE                    ✓      │    ✓
BEST_EFFORT                 ✗      │    ✓
```

图 3-2：QoS Reliability 兼容性

> 规则：Publisher 可靠级别必须 ≥ Subscriber 可靠级别

---

## P12 · QoS 配置代码

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

# 方式1：预定义
from rclpy.qos import qos_profile_sensor_data

# 方式2：自定义
qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=5
)
self.pub = self.create_publisher(Image, 'cam', qos)
```

---

## P13 · 执行器与回调组

```python
# 单线程（默认）
executor = SingleThreadedExecutor()

# 多线程并行
executor = MultiThreadedExecutor(num_threads=4)

# 回调组
from rclpy.callback_groups import ReentrantCallbackGroup
group = ReentrantCallbackGroup()
self.sub = self.create_subscription(
    ..., callback_group=group)
```

---

## P14 · 本章要点

1. Topic = 异步多对多发布-订阅
2. `create_publisher(msg_type, topic, qos)`
3. `create_subscription(msg_type, topic, callback, qos)`
4. 自定义消息在 CMake 包中定义 .msg
5. QoS 兼容性：RELIABLE → BEST_EFFORT ✓，反向 ✗
6. MultiThreadedExecutor + ReentrantCallbackGroup 实现并发

---

## P15 · 练习题

1. 发布 Twist 消息控制机器人 4
2. 订阅 /cmd_vel 并打印![alt text](images/image-1.png)
·补充 /cmd_vel 订阅节点
nano ~/my_ros2_ws/src/topic_demo/topic_demo/cmd_vel_subscriber.py
输入：
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(
            f'linear.x={msg.linear.x:.2f}, '
            f'angular.z={msg.angular.z:.2f}')
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CmdVelSubscriber())
    rclpy.shutdown()
编辑 topic_demo/setup.py，在 console_scripts 中添加：
'cmd_vel_sub = topic_demo.cmd_vel_subscriber:main',
          'square_driver = topic_demo.twist_square:main'
确认 topic_demo/package.xml 包含：
<exec_depend>geometry_msgs</exec_depend>
编译：
source /opt/ros/humble/setup.bash
cd ~/my_ros2_ws
colcon build --packages-select topic_demo --symlink-install
source install/setup.bash
运行时打开三个终端：
# 终端1
source ~/ros2_course_ws/install/setup.bash
ros2 launch robot_sim_demo_ros2 sim_bringup.launch.py
# 终端2
source ~/my_ros2_ws/install/setup.bash
ros2 run topic_demo cmd_vel_sub
# 终端3
source ~/my_ros2_ws/install/setup.bash
ros2 run topic_demo square_driver
终端2应打印 linear.x=0.20、angular.z=1.57 等速度。

3. 创建自定义 Person.msg 3.2![alt text](images/image-2.png)
·补充 Person.msg
mkdir -p ~/my_ros2_ws/src/sensor_interfaces/msg
nano ~/my_ros2_ws/src/sensor_interfaces/msg/Person.msg
输入：
string name
uint8 age
float32 height
在 sensor_interfaces/CMakeLists.txt 的接口列表中增加：
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
  "msg/Person.msg"
)
编译并检查：
source /opt/ros/humble/setup.bash
cd ~/my_ros2_ws
colcon build --packages-select sensor_interfaces
source install/setup.bash
ros2 interface show sensor_interfaces/msg/Person
测试：
# 终端1
source /opt/ros/humble/setup.bash
source ~/my_ros2_ws/install/setup.bash
ros2 interface show sensor_interfaces/msg/Person
ros2 topic echo /person_info sensor_interfaces/msg/Person
# 终端2
source /opt/ros/humble/setup.bash
source ~/my_ros2_ws/install/setup.bash
ros2 topic pub --once /person_info sensor_interfaces/msg/Person \
"{name: 'Li Ming', age: 20, height: 1.75}"

4. 测试 QoS 兼容性 3.3
5. 多线程 executor 并行测试![alt text](images/image-3.png)
创建 executor_demo.py：
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class ExecutorDemo(Node):
    def __init__(self):
        super().__init__('executor_demo')
        self.start = time.monotonic()
        group = ReentrantCallbackGroup()
        self.timer_a = self.create_timer(3.0, self.callback_a,
                                         callback_group=group)
        self.timer_b = self.create_timer(3.0, self.callback_b,
                                         callback_group=group)

    def work(self, name):
        now = time.monotonic() - self.start
        thread = threading.get_ident()
        self.get_logger().info(
            f'{name} 开始: {now:.2f}s, thread={thread}')
        time.sleep(2.0)
        now = time.monotonic() - self.start
        self.get_logger().info(
            f'{name} 结束: {now:.2f}s, thread={thread}')

    def callback_a(self):
        self.work('A')

    def callback_b(self):
        self.work('B')


def main(args=None):
    rclpy.init(args=args)
    node = ExecutorDemo()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
在 setup.py 添加：
'executor_demo = topic_demo.executor_demo:main',
然后执行：
cd ~/my_ros2_ws
colcon build --packages-select topic_demo --symlink-install
source install/setup.bash
ros2 run topic_demo executor_demo
---

## P16 · 下章预告

**第 4 章：服务通信（Services）**

- 请求-响应模式
- 同步 vs 异步调用
- 超时与重试机制
