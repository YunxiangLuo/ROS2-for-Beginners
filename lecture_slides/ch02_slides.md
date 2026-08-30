# 第2章 PPT：ROS 2 核心编程基础

> 共 15 页，标注页码 · 图号与教学文档对应

---

## P1 · 标题页

**ROS 2 核心编程基础**

- 课程：ROS2 Python 编程
- 章节：第 2 章
- 课时：2 课时

---

## P2 · 本课学习目标

- 掌握 ament_python 包的标准目录结构与 package.xml 配置
- 理解 setup.py 中 entry_points 的注册机制
- 掌握最小节点模板与节点生命周期流程
- 理解节点名称、命名空间与重映射规则
- 掌握分级日志系统的使用
- 熟悉 CLI 命令、rqt_graph 与 RViz2 工具链

---

## P3 · Python 包标准结构

```
my_robot_pkg/                     # 包根目录
├── package.xml                   # 包元数据（名称、版本、依赖）
├── setup.py                      # 安装脚本
├── setup.cfg                     # 安装配置
├── resource/
│   └── my_robot_pkg              # ament index 标记文件
├── my_robot_pkg/                 # Python 源码
│   ├── __init__.py
│   └── my_node.py                # 节点代码
├── launch/                       # Launch 文件目录
│   └── demo.launch.py
└── test/                         # 测试代码
    ├── __init__.py
    └── test_my_node.py
```

图 2-1：ROS 2 Python 包的标准目录结构

- 创建命令：`ros2 pkg create my_pkg --build-type ament_python --dependencies rclpy`

---

## P4 · package.xml 配置

```xml
<package format="3">
  <name>my_robot_pkg</name>
  <version>0.1.0</version>
  <description>My first ROS 2 Python package</description>
  <maintainer email="student@example.com">Student Name</maintainer>
  <license>Apache-2.0</license>

  <!-- 运行时依赖 -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- depend 标签必须与实际 import 的依赖一致

---

## P5 · setup.py 安装入口

```python
setup(
    name='my_robot_pkg',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/my_robot_pkg']),
        ('share/my_robot_pkg', ['package.xml']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'my_node = my_robot_pkg.my_node:main',
        ],
    },
)
```

- `entry_points` 是关键配置：`'可执行文件名 = 包名.模块名:函数名'`

---

## P6 · 最小节点模板

```python
class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node')  # 节点名称
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
        self.get_logger().info('节点已启动')

    def timer_callback(self):
        self.count += 1
        self.get_logger().info(f'Hello ROS 2! Count: {self.count}')

def main(args=None):
    rclpy.init(args=args)
    node = MyFirstNode()
    try:
        rclpy.spin(node)         # 事件循环（阻塞）
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

程序 2-1：最小节点模板——「初始化→创建→spin→销毁→关闭」标准流程

---

## P7 · 节点名称与命名空间

```python
# 方式1：构造函数中设置
node = rclpy.create_node('node_name', namespace='my_ns')

# 方式2：命令行参数重映射
# ros2 run my_pkg my_node --ros-args -r __ns:=/new_ns \
#                          -r __node:=new_name
```

- 命名规则：节点名仅含字母、数字、下划线，不能含空格
- 完整节点标识 = `{namespace}/{node_name}`，如 `/my_ns/node_name`
- 不同命名空间下的节点相互隔离；同名节点在同一命名空间下无法共存
- 节点名与可执行文件名是两回事，同一可执行文件可重映射出多个节点实例

---

## P8 · 分级日志系统

| 日志级别 | Python API | 用途 |
|---------|-----------|------|
| DEBUG | `debug()` | 调试信息（默认不输出） |
| INFO | `info()` | 一般信息 |
| WARN | `warn()` | 警告信息 |
| ERROR | `error()` | 错误信息 |
| FATAL | `fatal()` | 致命错误 |

```python
self.get_logger().set_level(LoggingSeverity.DEBUG)

# 节流日志：每秒最多输出一次
self.get_logger().info('消息', throttle_duration_sec=1)

# 一次性日志：整个生命周期只输出一次
self.get_logger().info('消息', once=True)
```

---

## P9 · 命令行工具链

```bash
colcon build --packages-select my_pkg --symlink-install
source install/setup.bash
ros2 run my_pkg my_node

ros2 node list               # 列出节点
ros2 node info /node         # 节点详情（发布/订阅/QoS）
ros2 topic list              # 列出话题
ros2 topic echo /topic       # 监听话题消息
ros2 topic hz /topic         # 话题频率
ros2 pkg list                # 列出包
ros2 pkg xml my_robot_pkg    # 查看包 XML
```

- `ros2 node info` 是排查「节点间连不上」的第一工具

---

## P10 · rqt_graph 可视化

```bash
rqt_graph                      # 或 ros2 run rqt_graph rqt_graph
# 安装：sudo apt install ros-humble-rqt-graph
```

图 2-2：rqt_graph 界面示意图

- 椭圆 = 节点，矩形 = 话题，连线 = 发布/订阅关系
- 勾选「Nodes/Topics (all)」查看完整通信图

---

## P11 · RViz2 可视化

```bash
rviz2                          # 或 rviz2 -d /path/to/config.rviz
# 安装：sudo apt install ros-humble-rviz2
```

图 2-3：RViz2 界面布局

- 左侧 Displays 面板：添加 RobotModel、TF、LaserScan 等显示项
- 右侧 Views 面板：Orbit、TopDownOrtho 等多种视角

---

## P12 · 生命周期节点进阶

```python
from rclpy.lifecycle import LifecycleNode

class MyNode(LifecycleNode):
    def on_configure(self, state):   pass
    def on_activate(self, state):    pass
    def on_deactivate(self, state):  pass
```

```bash
ros2 lifecycle get /node                  # 查询状态
ros2 lifecycle set /node configure        # 触发迁移
```

- 只有全部组件进入 Active 后，任务级应用才允许发送控制指令
- 对照官方 lifecycle/talker.py 示例重写状态回调

---

## P13 · 本章要点

1. ament_python 包 = package.xml + setup.py + 源码目录
2. entry_points 把 Python 函数注册为 ROS 2 可执行文件
3. 节点流程：init → create_node → spin → destroy → shutdown
4. 日志分级 DEBUG/INFO/WARN/ERROR/FATAL，支持节流与一次性
5. ros2 node/topic/pkg 命令是调试核心手段
6. rqt_graph 看通信拓扑，RViz2 看机器人与传感器数据

---

## P14 · 练习题

1. 创建 hello_pkg 包，含 hello_node 节点，每秒输出一句问候语
2. 添加 `--ros-args --log-level DEBUG`，观察不同日志级别输出差异
3. 运行两个 hello_node，用 `-r __node:=hello2` 重命名，ros2 node list 验证
4. 用 ros2 node info 查看 talker 的发布/订阅信息，画出通信拓扑图
5. 启动 talker/listener，用 rqt_graph 可视化通信关系
6. 安装启动 RViz2，认识显示面板和视角控制

> 提示：练习 3 验证节点名与可执行文件名可分离；练习 4 可结合 `ros2 topic info` 交叉验证

---

## P15 · 下章预告

**第 3 章：话题通信**

- 发布者 / 订阅者编程模型（Publisher / Subscription）
- 自定义消息类型 Person.msg
- 多线程 executor 并行处理