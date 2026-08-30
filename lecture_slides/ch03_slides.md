# 第3章 PPT：话题通信（Topics）

> 共 16 页，标注页码 · 图号与教学文档对应

---

## P1 · 标题页

**话题通信（Topics）**

- 课程：ROS2 Python 编程
- 章节：第 3 章
- 课时：2 课时

---

## P2 · 本课学习目标

- 理解发布-订阅模型：异步、多对多、完全解耦
- 掌握 create_publisher / create_subscription API
- 熟悉 std_msgs、sensor_msgs、geometry_msgs 标准消息
- 掌握自定义消息 .msg 接口的创建与使用
- 理解 QoS 可靠性兼容性规则及违约现象
- 使用多线程执行器与回调组提升并行处理能力

---

## P3 · 话题通信架构

```
Publisher 1 ─────┐
                 ├──►  Topic: "/camera/image"  ──► Subscriber A
Publisher 2 ─────┘                                  Subscriber B
                                                    Subscriber C
```

图 3-1：话题通信的多对多发布-订阅模型

- 发布者与订阅者完全解耦，互不知晓对方的存在
- 话题名 + 消息类型 = 通信双方的唯一协议

---

## P4 · Python Publisher API

```python
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        # 创建发布者：话题名 "chatter"，队列深度 10
        self.publisher = self.create_publisher(
            String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'发布: "{msg.data}"')
        self.count += 1
```

程序 3-1：Publisher 完整示例——`create_publisher(消息类型, 话题名, QoS 队列深度)`

---

## P5 · Python Subscriber API

```python
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String, 'chatter',      # 话题名
            self.listener_callback, # 回调函数
            10)                     # QoS 队列深度

    def listener_callback(self, msg):
        self.get_logger().info(f'收到: "{msg.data}"')
```

程序 3-2：Subscriber 完整示例——回调签名必须为 `callback(msg)`

- 回调中 `msg` 对象在回调返回后即失效，切勿保存引用
- 不要用 while + time.sleep 代替 create_timer——会阻塞回调

---

## P6 · 标准消息类型

```
std_msgs/                 # 基础类型
├── String / Int32 / Int64 / Float32 / Float64
├── Bool / Empty / Header (stamp + frame_id)

sensor_msgs/              # 传感器消息
├── Image (rgb, depth) / LaserScan / PointCloud2
├── Imu / Joy

geometry_msgs/            # 几何消息
├── Twist (linear + angular) / Pose / Vector3 / Quaternion
```

- 自定义消息类型必须在接口包中定义（见 P8–P9）

---

## P7 · 查看消息定义

```bash
ros2 interface show std_msgs/msg/String
# 输出：string data

ros2 interface show geometry_msgs/msg/Twist
# Vector3 linear
#   float64 x / y / z
# Vector3 angular
#   float64 x / y / z

ros2 interface proto std_msgs/msg/String   # 查看消息属性
```

```bash
ros2 topic list -t     # 含类型
ros2 topic echo /topic # 查看消息流
ros2 topic info /topic # 发布/订阅计数与 QoS
ros2 topic hz /topic   # 测频率
ros2 topic bw /topic   # 测带宽
```

- 类比「广播电台」：发布者广播、订阅者收听、频道与节目单必须一致

---

## P8 · 自定义消息接口包

```
custom_interfaces/
├── CMakeLists.txt
├── package.xml
└── msg/
    └── SensorData.msg
```

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg")
```

```python
# msg/SensorData.msg
float64 temperature    # 温度 (℃)
float64 humidity       # 湿度 (%)
float64 pressure       # 气压 (hPa)
string device_id       # 设备ID
```

- 接口包仅支持 CMake 构建、单独建包独立编译，避免连锁重编译

---

## P9 · 使用自定义消息

```python
# package.xml 中添加：
# <exec_depend>custom_interfaces</exec_depend>

from custom_interfaces.msg import SensorData

msg = SensorData()
msg.temperature = 25.5
msg.humidity = 60.0
msg.pressure = 1013.25
msg.device_id = 'sensor_01'
self.publisher.publish(msg)
```

- IDL 支持默认值、数组、嵌套类型；.msg 可引用其他接口包类型（如 `geometry_msgs/Point position`）
- 编译后用 `ros2 interface show` 验证

---

## P10 · QoS 兼容性规则

```
           Publisher ╲ Subscriber │ RELIABLE │ BEST_EFFORT
           ───────────────────────┼──────────┼─────────────
           RELIABLE               │    ✓     │     ✓
           BEST_EFFORT            │    ✗     │     ✓
```

图 3-2：QoS Reliability 兼容性矩阵

- Publisher 的可靠级别必须 >= Subscriber 的可靠级别
- 控制指令像电话（RELIABLE 保证接通），传感器流像对讲机（BEST_EFFORT 丢一条接着听）
- 日志出现 `Incompatible QoS policies` 即两端不兼容，用 `ros2 topic info -v` 排障

---

## P11 · Python 中配置 QoS

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, \
    DurabilityPolicy, HistoryPolicy

# 方式1：预定义配置（传感器数据）
self.publisher = self.create_publisher(
    Image, 'camera/image', qos_profile_sensor_data)

# 方式2：自定义 QoS
custom_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5)
self.subscription = self.create_subscription(
    LaserScan, 'scan', self.callback, custom_qos)
```

- 默认档案：sensor_data (BEST_EFFORT+KEEP_LAST(5))、system_default (RELIABLE+KEEP_LAST(10))、params

---

## P12 · 多线程执行器

```python
from rclpy.executors import \
    SingleThreadedExecutor, MultiThreadedExecutor

# 单线程（默认）：顺序执行所有回调
executor = SingleThreadedExecutor()
executor.add_node(node);  executor.spin()

# 多线程：并行执行回调，提升吞吐量
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node);  executor.spin()
```

- 多节点共用同一 executor，spin 统一驱动

---

## P13 · 回调组

```python
from rclpy.callback_groups import \
    MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# 互斥回调组：组内回调串行执行（默认行为）
group1 = MutuallyExclusiveCallbackGroup()

# 可重入回调组：组内回调可并行执行
group2 = ReentrantCallbackGroup()

self.sub = self.create_subscription(
    Image, 'camera', self.callback, 10,
    callback_group=group2)
```

- 回调组 + 多线程执行器组合使用才能发挥并行能力

---

## P14 · 本章要点

1. 话题通信：异步、多对多、发布者与订阅者完全解耦
2. create_publisher(类型, 话题, QoS) / create_subscription(类型, 话题, 回调, QoS)
3. 标准消息分类：std_msgs 基础、sensor_msgs 传感器、geometry_msgs 几何
4. 自定义消息在独立 CMake 接口包中定义 .msg，编译后 import 使用
5. QoS 兼容性：RELIABLE 发布者可与 BEST_EFFORT 订阅者通信，反之不行
6. 多线程执行器 + 回调组并行处理，提高吞吐量

---

## P15 · 练习题

1. 编写 Publisher 发布 Twist 消息控制机器人运动（含线速度、角速度）
2. 编写 Subscriber 订阅 /cmd_vel，打印收到的速度指令
3. 创建自定义接口包，定义含姓名、年龄、身高字段的 Person.msg
4. 基于 Person.msg 编写发布者/订阅者：`ros2 run sensor_pub person_pub` 与 `person_sub`
5. 测试 RELIABLE/BEST_EFFORT 兼容性：`ros2 run topic_demo qos_pub`，再用 `ros2 topic echo --qos-reliability best_effort` 观察（RELIABLE 发布者 + BEST_EFFORT 订阅者可行）
6. MultiThreadedExecutor 运行 2 Publisher + 2 Subscriber（executor_single / executor_multi 对比线程与耗时）

> 提示：练习 4 需先 `colcon build --packages-select sensor_interfaces sensor_pub`；练习 6 用 `ros2 node list` 验证四个节点

---

## P16 · 下章预告

**第 4 章：服务通信（Services）**

- 客户端 / 服务器模型（Client / Service）
- 请求-响应（request-response）同步通信模式
- 自定义 .srv 接口与异步调用