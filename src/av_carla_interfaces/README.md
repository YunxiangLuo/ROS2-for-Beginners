# av_carla_interfaces — 自定义接口包

定义工作空间共用的消息(.msg)、服务(.srv)与动作(.action), 供其他包的 C++/Python 节点使用。

## 目录结构

```
av_carla_interfaces/
├── CMakeLists.txt        # rosidl 接口生成清单
├── package.xml
├── msg/
│   ├── ControlCmd.msg            # 车辆控制指令(throttle/brake/steer/gear...)
│   ├── EgoState.msg              # 自车状态(speed/pose/twist)
│   ├── Waypoint.msg / WaypointArray.msg
│   ├── PerceptionObject.msg / PerceptionObjectArray.msg
│   ├── Cluster.msg / ClusterArray.msg      # LiDAR 聚类结果
│   └── CollisionEvent.msg                  # 碰撞事件
├── srv/PlanPath.srv
├── action/Navigate.action
└── test/test_msg_consistency.py  # 接口一致性静态测试
```

## 安装与编译

```bash

cd <工作空间根目录>

colcon build --packages-select av_carla_interfaces

source install/setup.bash
```

验证生成的接口:

```bash
ros2 interface show av_carla_interfaces/msg/ControlCmd
ros2 interface show av_carla_interfaces/action/Navigate
```

## 测试方法

```bash

cd src/av_carla_interfaces

python -m pytest test -q
```

测试内容: CMakeLists 注册的接口文件存在性、.msg 字段语法、C++ 源码
`msg->field` 引用与 .msg 定义一致性、Python 包 import 的消息已注册、
Waypoint/EgoState 字段与用法匹配。

## 运行结果

```text
$ cd src/av_carla_interfaces && python -m pytest test -q
......                                                                   [100%]
6 passed in 0.05s
```

> 说明: 本机(Windows)未安装 ROS2/CARLA, 无法截取仿真运行画面,
> 运行结果以**真实终端输出**代替截图; 全部输出均可按上述命令复现。

## 本次修复记录

1. `CMakeLists.txt` 引用了不存在的 `msg/ControlCmd.msg` → 新增该消息定义;
2. `av_perception_py` 引用的 `Cluster/ClusterArray` 与 `av_safety_monitor`
   引用的 `CollisionEvent` 未定义 → 补齐 3 个消息并注册到 CMakeLists;
3. 新增 `test/test_msg_consistency.py` 静态一致性测试, 防止消息定义与
   C++/Python 引用再次漂移。
