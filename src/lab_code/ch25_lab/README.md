# 第25章 实验代码：全局路径规划与地图导航

## 文件说明

| 文件名 | 功能 | 依赖 |
|--------|------|------|
| `global_planner.py` | A\* 全局路径规划器，在 CARLA 路网图中搜索最优路径 | carla, numpy, heapq |
| `waypoint_pub.py` | ROS2 节点，通过 CARLA Waypoint API 生成连续路径并发布 | carla, rclpy, nav_msgs |
| `map_visualizer.py` | 可视化 CARLA 地图 OpenDRIVE 路网和拓扑结构 | carla, matplotlib, xml.etree |

## 使用方法

```bash
# 1. 启动 CARLA
./CarlaUE4.sh -quality-level=Low

# 2. 运行地图可视化
python map_visualizer.py

# 3. 运行全局路径规划
python global_planner.py

# 4. 运行 ROS2 Waypoint 节点（需先 source ROS 2）
python3 src/lab_code/ch25_lab/waypoint_pub.py
```

## 依赖安装

```bash
pip install carla matplotlib numpy
# ROS 2 Jazzy 依赖
sudo apt install ros-jazzy-nav-msgs ros-jazzy-geometry-msgs
```

---

## 安装与编译

```bash

pip install numpy matplotlib
```

## 运行方法

```bash
# 纯算法验证(无需 CARLA): 在 CARLA 路网拓扑上运行 A* 全局规划
python global_planner.py --host localhost --port 2000 --start-index 0
python global_planner.py --dijkstra            # Dijkstra 对比
python map_visualizer.py --export              # 导出 OpenDRIVE 并绘制路网图
python waypoint_pub.py                          # 航点发布(需 ROS2)
```

## 验证

在 CARLA 路网已加载时，分别运行 `global_planner.py` 与 `--dijkstra` 参数，可对比两种搜索结果；`waypoint_pub.py` 需要已加载 ROS 2 Jazzy 环境。本目录未提供独立离线测试套件。
