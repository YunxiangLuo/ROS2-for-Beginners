# av_planning_py — 规划节点

A* 栅格全局规划、航点生成与 Navigate 动作服务。

## 目录结构

```
av_planning_py/
├── setup.py / package.xml
├── config/planner_params.yaml
├── resource/av_planning_py
├── av_planning_py/
│   ├── global_planner.py      # AStarPlanner(障碍膨胀 + 8/4 邻域 A*) + ROS 节点
│   ├── waypoint_generator.py  # 目标点 -> 等距航点序列(WaypointArray)
│   └── planning_server.py     # /navigate 动作服务(Navigate.action)
└── test/test_astar.py
```

## 安装与编译

```bash
cd <工作空间根目录>
colcon build --packages-select av_carla_interfaces av_planning_py
source install/setup.bash
```

## 运行方法

```bash
ros2 run av_planning_py global_planner       # 订阅 /map, /goal_pose -> /plan
ros2 run av_planning_py waypoint_generator   # 订阅 /goal_pose -> /waypoints
ros2 run av_planning_py planning_server      # 动作服务 /navigate

# 发送导航目标
ros2 action send_goal /navigate av_carla_interfaces/action/Navigate "{target_location: '20.0, 30.0'}"
```

## 测试方法

```bash
cd src/av_planning_py
python -m pytest test -q
```

## 运行结果

```text
$ cd src/av_planning_py && python -m pytest test -q
............                                                           [100%]
12 passed in 0.05s
```

覆盖: 空地图寻路、路径连续性、绕墙、全封锁不可达、起终点在障碍/图外、
4 邻域模式、障碍膨胀区禁入、栅格/世界坐标往返、膨胀半径小于分辨率防护。

> 说明: 本机(Windows)未安装 ROS2/CARLA, 无法截取仿真运行画面,
> 运行结果以**真实终端输出**代替截图; 全部输出均可按上述命令复现。

## 本次修复记录

1. `AStarPlanner.__init__` 未初始化 `resolution`/`inflated_grid`(`set_map`
   前调用坐标转换会 AttributeError) → 补齐初始化;
2. `_inflate_obstacles` 在膨胀半径小于栅格分辨率时按 0 处理仍走全图循环 →
   增加早退防护并返回原始占用副本;
3. 新增 12 个 A* 单元测试。
