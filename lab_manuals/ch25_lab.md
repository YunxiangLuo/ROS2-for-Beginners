# 第25章 实验手册：全局路径规划与地图导航

> **对应理论章节**：第39章《全局路径规划与地图导航》  
> **实验课时**：3 课时  
> **实验代码**：`src/lab_code/ch25_lab/`

---

## 练习 25.1：获取 CARLA 地图 OpenDRIVE 并可视化路网

### 实验目标
掌握从 CARLA 模拟器获取 OpenDRIVE 地图数据的方法，并可视化道路网络结构。

### 实验步骤

1. **启动 CARLA 服务器**
   ```bash
   # 在 CARLA 安装目录下
   ./CarlaUE4.sh -quality-level=Low   # Linux
   # 或
   CarlaUE4.exe -quality-level=Low     # Windows
   ```

2. **编写 Python 脚本获取 OpenDRIVE**
   ```python
   import carla
   import time

   client = carla.Client('localhost', 2000)
   client.set_timeout(10.0)
   world = client.get_world()
   carla_map = world.get_map()

   # 导出 OpenDRIVE 到文件
   opendrive = carla_map.to_opendrive()
   with open("carla_map.xodr", "w", encoding="utf-8") as f:
       f.write(opendrive)
   print(f"OpenDRIVE 已导出，长度: {len(opendrive)} 字符")
   ```

3. **可视化路网拓扑**
   使用 `carla.Map` 的拓扑方法绘制路网图：
   ```python
   import matplotlib.pyplot as plt
   import numpy as np

   # 获取道路拓扑
   topology_list = carla_map.get_topology()
   print(f"道路拓扑边数: {len(topology_list)}")

   # 绘制道路网络
   plt.figure(figsize=(12, 8))
   for edge in topology_list:
       wp_start, wp_end = edge
       start_loc = wp_start.transform.location
       end_loc = wp_end.transform.location
       plt.plot([start_loc.x, end_loc.x],
                [start_loc.y, end_loc.y], 'b-', alpha=0.5, linewidth=0.5)
       plt.plot(start_loc.x, start_loc.y, 'g.', markersize=1)
       plt.plot(end_loc.x, end_loc.y, 'r.', markersize=1)

   plt.axis('equal')
   plt.title('CARLA 道路拓扑网络')
   plt.xlabel('X (m)')
   plt.ylabel('Y (m)')
   plt.grid(True, alpha=0.3)
   plt.savefig('road_topology.png', dpi=150)
   plt.show()
   ```

4. **分析 OpenDRIVE 结构**
   解析 XML 提取道路信息：
   ```python
   import xml.etree.ElementTree as ET
   root = ET.fromstring(opendrive)

   roads = root.findall('.//road')
   print(f"道路总数: {len(roads)}")
   for road in roads[:5]:
       road_id = road.get('id')
       road_name = road.get('name')
       road_len = road.get('length')
       lanes = road.find('.//laneSection')
       left_lanes = lanes.find('left') if lanes is not None else None
       right_lanes = lanes.find('right') if lanes is not None else None
       n_left = len(left_lanes.findall('lane')) if left_lanes is not None else 0
       n_right = len(right_lanes.findall('lane')) if right_lanes is not None else 0
       print(f"  道路 {road_id}: {road_name}, 长度={road_len}m, "
             f"左车道数={n_left}, 右车道数={n_right}")
   ```

### 预期输出
- `carla_map.xodr` 文件（OpenDRIVE XML）
- `road_topology.png` 路网拓扑图
- 控制台打印的道路统计信息

---

## 练习 25.2：实现 A\* 全局路径规划

### 实验目标
在网格化的路网中实现 A\* 搜索算法，并在 CARLA 中可视化规划路径。

### 实验步骤

1. **构建路网图**
   将练习 25.1 获取的道路拓扑转换为可供搜索的图结构：
   ```python
   class RoadGraph:
       def __init__(self):
           self.nodes = {}     # node_id -> (x, y)
           self.edges = {}     # node_id -> [(neighbor_id, cost)]

       def add_node(self, node_id, x, y):
           self.nodes[node_id] = (x, y)

       def add_edge(self, from_id, to_id, cost=None):
           if cost is None:
               x1, y1 = self.nodes[from_id]
               x2, y2 = self.nodes[to_id]
               cost = np.sqrt((x2-x1)**2 + (y2-y1)**2)
           self.edges.setdefault(from_id, []).append((to_id, cost))

   def build_graph_from_topology(carla_map):
       graph = RoadGraph()
       topology = carla_map.get_topology()

       node_id_map = {}
       next_id = 0

       for wp_start, wp_end in topology:
           for wp in [wp_start, wp_end]:
               loc = wp.transform.location
               key = (round(loc.x, 2), round(loc.y, 2))
               if key not in node_id_map:
                   node_id_map[key] = next_id
                   graph.add_node(next_id, loc.x, loc.y)
                   next_id += 1

           start_key = (round(wp_start.transform.location.x, 2),
                        round(wp_start.transform.location.y, 2))
           end_key = (round(wp_end.transform.location.x, 2),
                      round(wp_end.transform.location.y, 2))
           graph.add_edge(node_id_map[start_key], node_id_map[end_key])

       return graph, node_id_map
   ```

2. **实现 A\* 搜索**
   ```python
   import heapq

   def heuristic(a_id, b_id, nodes):
       x1, y1 = nodes[a_id]
       x2, y2 = nodes[b_id]
       return np.sqrt((x2-x1)**2 + (y2-y1)**2)

   def a_star_search(graph, start_id, goal_id):
       open_set = [(0, start_id)]
       came_from = {}
       g_score = {node_id: float('inf') for node_id in graph.nodes}
       g_score[start_id] = 0
       f_score = {node_id: float('inf') for node_id in graph.nodes}
       f_score[start_id] = heuristic(start_id, goal_id, graph.nodes)

       in_open = {start_id}

       while open_set:
           _, current = heapq.heappop(open_set)
           in_open.discard(current)

           if current == goal_id:
               path = []
               while current in came_from:
                   path.append(current)
                   current = came_from[current]
               path.append(start_id)
               return path[::-1]

           for neighbor, cost in graph.edges.get(current, []):
               tentative_g = g_score[current] + cost
               if tentative_g < g_score[neighbor]:
                   came_from[neighbor] = current
                   g_score[neighbor] = tentative_g
                   f_score[neighbor] = g_score[neighbor] + \
                       heuristic(neighbor, goal_id, graph.nodes)
                   if neighbor not in in_open:
                       heapq.heappush(open_set, (f_score[neighbor], neighbor))
                       in_open.add(neighbor)

       return []  # 无路径
   ```

3. **在 CARLA 中可视化规划路径**
   ```python
   def visualize_path(world, path, graph, color=carla.Color(0, 255, 0)):
       waypoints = []
       for node_id in path:
           x, y = graph.nodes[node_id]
           loc = carla.Location(x, y, 0.5)
           waypoint = world.get_map().get_waypoint(loc)
           if waypoint:
               waypoints.append(waypoint.transform.location)

       # 使用 DebugHelper 绘制
       debug = world.debug
       for i in range(len(waypoints) - 1):
           debug.draw_line(
               waypoints[i],
               waypoints[i+1],
               thickness=0.3,
               color=color,
               life_time=30.0
           )
   ```

### 思考题
1. 当路网中存在大量节点时，A\* 的 open_set 会包含多少节点？与 Dijkstra 对比如何？
2. 在 CARLA 的多层道路（如高架桥与地面路重叠）场景下，Waypoint 如何区分不同层？

---

## 练习 25.3：使用 Waypoint 发布器生成导航路径

### 实验目标
创建一个 ROS2 节点，通过 CARLA Waypoint API 生成连续导航路径，并发布为 ROS2 消息。

### 实验步骤

1. **创建 ROS2 包（若需要）**
   ```bash
   cd ~/ros2_course_ws/src
   ros2 pkg create ch25_waypoint_nav --build-type ament_python \
       --dependencies rclpy geometry_msgs nav_msgs carla_msgs
   ```

2. **实现 Waypoint 路径生成节点**
   参考本章 `waypoint_pub.py` 代码，重点实现：
   - 从起点到目标点的 Waypoint 链生成
   - 路径转换为 `nav_msgs/Path` 消息
   - 以 0.5Hz 频率发布路径

3. **在 Rviz2 中可视化路径**
   ```bash
   ros2 launch ch25_waypoint_nav waypoint_nav.launch.py
   rviz2 -d waypoint_nav.rviz
   ```
   在 Rviz2 中添加 `Path` 显示类型，订阅 `/planned_path` 话题。

4. **将路径输出为 KML（可选）**
   方便在 Google Earth 中查看：
   ```python
   def export_to_kml(waypoints, filename="route.kml"):
       kml_header = """<?xml version="1.0" encoding="UTF-8"?>
   <kml xmlns="http://www.opengis.net/kml/2.2">
     <Document>
       <Placemark>
         <LineString>
           <coordinates>"""
       kml_footer = """</coordinates>
         </LineString>
       </Placemark>
     </Document>
   </kml>"""

       coords = "\n".join(
           f"{wp.transform.location.x},{wp.transform.location.y},0"
           for wp in waypoints
       )
       with open(filename, "w") as f:
           f.write(kml_header + coords + kml_footer)
       print(f"KML 已导出: {filename}")
   ```

### 预期结果
- 在 Rviz2 中看到绿色的规划路径
- 路径沿车道中心线分布
- 在路口处根据目标方向选择正确出口

### 挑战任务
在 CARLA 中运行 ego-vehicle，使其沿生成的 Waypoint 路径自主行驶（可使用 CARLA 的 `WaypointFollower` 或自定义 PID 控制器）。

---

## 实验报告要求

1. 提交练习 25.1 生成的路网拓扑图 `road_topology.png`
2. 提交 A\* 路径规划代码及测试结果截图
3. 提交 ROS2 节点运行截图（Rviz2 中显示路径）
4. 撰写实验总结（不少于 200 字），对比 Dijkstra 与 A\* 在 CARLA 路网中的实际表现
