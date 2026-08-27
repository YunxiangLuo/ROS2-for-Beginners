# 第2章 PPT：ROS 2 核心编程基础

> 共 14 页，标注页码

---

## P1 · 标题页

**ROS 2 核心编程基础**

- 课程：ROS2 Python 编程
- 章节：第 2 章
- 课时：2 课时

---

## P2 · 本课学习目标

- 掌握 ROS 2 Python 包的标准结构
- 编写第一个 ROS 2 Python 节点
- 理解节点命名空间与重映射
- 使用日志系统调试程序
- 熟练使用命令行工具和可视化工具

---

## P3 · Python 包标准结构

```
my_robot_pkg/
├── package.xml        ← 包元数据
├── setup.py           ← 安装入口（entry_points）
├── setup.cfg
├── resource/
│   └── my_robot_pkg
├── my_robot_pkg/      ← 源码目录
│   ├── __init__.py
│   └── my_node.py
├── launch/
└── test/
```

图 2-1：包结构

---

## P4 · package.xml 关键配置

```xml
<package format="3">
  <name>my_robot_pkg</name>
  <version>0.1.0</version>
  <exec_depend>rclpy</exec_depend>     ← 运行时依赖
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## P5 · setup.py entry_points

```python
entry_points={
    'console_scripts': [
        'my_node = my_robot_pkg.my_node:main',
        #  ↑ 命令         ↑ 包.模块:函数
    ],
},
```

> `ros2 run my_robot_pkg my_node` 实际调用 `my_robot_pkg.my_node:main()`

---

## P6 · 最小节点结构

```
程序 2-1：最小节点模板

def main():
    rclpy.init()          ← ① 初始化
    node = MyNode()       ← ② 创建节点
    rclpy.spin(node)      ← ③ 事件循环（阻塞）
    node.destroy_node()   ← ④ 销毁节点
    rclpy.shutdown()      ← ⑤ 关闭
```

---

## P7 · 最小节点代码

```python
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.timer = self.create_timer(1.0, self.callback)

    def callback(self):
        self.get_logger().info('Hello ROS 2!')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MyFirstNode())
    rclpy.shutdown()
```

---

## P8 · 节点命名空间

```python
# 方式1：构造函数指定
super().__init__('node_name', namespace='my_ns')

# 方式2：命令行重映射
$ ros2 run pkg node --ros-args \
    -r __ns:=/new_ns \
    -r __node:=renamed
```

---

## P9 · 日志系统分级

| 级别 | 方法 | 用途 |
|------|------|------|
| DEBUG | `.debug()` | 排查问题 |
| INFO | `.info()` | 一般信息 |
| WARN | `.warn()` | 警告 |
| ERROR | `.error()` | 错误 |
| FATAL | `.fatal()` | 致命 |

```python
self.get_logger().info('msg', throttle_duration_sec=1)  # 节流
self.get_logger().info('msg', once=True)                 # 仅一次
```

---

## P10 · 命令行工具

```bash
ros2 node list                  # 列出节点
ros2 node info /node_name       # 节点详情
ros2 topic list                 # 列出话题
ros2 topic echo /topic          # 监听话题
ros2 topic hz /topic            # 查看频率
ros2 pkg list                   # 列出包
```

---

## P11 · rqt_graph 可视化

启动命令：
```bash
rqt_graph
```

- 椭圆 = 节点
- 矩形 = 话题
- 箭头 = 通信方向

图 2-2：rqt_graph 通信拓扑

---

## P12 · RViz2 可视化

```bash
rviz2                           # 默认启动
rviz2 -d config.rviz            # 指定配置
```

左侧 Displays 面板：
- RobotModel（URDF 模型）
- TF（坐标系）
- LaserScan（激光数据）
- Camera（图像）

图 2-3：RViz2 界面

---

## P13 · 本章要点

1. Python 包通过 `entry_points` 注册可执行文件
2. 节点标准流程：init → spin → shutdown
3. 日志系统支持分级输出、节流、一次性输出
4. `ros2 node/topic/pkg` 是核心调试命令
5. rqt_graph 查看通信拓扑，RViz2 查看传感器数据

---

## P14 · 练习题

1. 创建 hello_pkg 包，包含 hello_node 节点 3.1
2. 测试日志节流和一次性输出 3.1\3.2
3. 使用命名空间运行多个同名节点
ros2 run hello_pkg hello_node --ros-args -r __ns:=/robot1(终端1)
ros2 run hello_pkg hello_node --ros-args -r __ns:=/robot2(终端2)
ros2 node list(终端3)
应看到：
/robot1/hello_node
/robot2/hello_node
![alt text](images/image.png)
4. 用 rqt_graph 查看 talker/listener 通信 3.3
5. 启动 RViz2 熟悉界面 3.3
