"""global_planner.py — A* 全局路径规划器

在 CARLA 地图道路拓扑图上执行 A* 搜索，找到从起点到目标点的
最短路径，并在 CARLA Simulator 中可视化。

用法:
    python global_planner.py [--host HOST] [--port PORT] [--start-index START] [--goal-index GOAL]

依赖:
    carla, numpy
"""

import argparse
import heapq
import math
import random
import sys

import numpy as np

try:
    import carla
except ImportError:
    raise ImportError("carla 模块未找到。请确保已安装 CARLA Python API。")


class RoadGraph:
    """基于 CARLA 拓扑构建的路网图结构"""

    def __init__(self):
        self.nodes = {}
        self.edges = {}
        self.node_locations = {}

    def add_node(self, node_id, location):
        self.nodes[node_id] = (location.x, location.y, location.z)
        self.node_locations[node_id] = location

    def add_edge(self, from_id, to_id, cost=None):
        if cost is None:
            x1, y1, _ = self.nodes[from_id]
            x2, y2, _ = self.nodes[to_id]
            cost = math.hypot(x2 - x1, y2 - y1)
        self.edges.setdefault(from_id, []).append((to_id, cost))

    def get_neighbors(self, node_id):
        return self.edges.get(node_id, [])


def build_graph_from_topology(carla_map):
    """从 CARLA 地图拓扑构建 RoadGraph"""
    graph = RoadGraph()
    topology = carla_map.get_topology()

    node_id_map = {}
    next_id = 0

    for wp_start, wp_end in topology:
        for wp in (wp_start, wp_end):
            loc = wp.transform.location
            key = (round(loc.x, 2), round(loc.y, 2), round(loc.z, 2))
            if key not in node_id_map:
                node_id_map[key] = next_id
                graph.add_node(next_id, loc)
                next_id += 1

        start_key = (
            round(wp_start.transform.location.x, 2),
            round(wp_start.transform.location.y, 2),
            round(wp_start.transform.location.z, 2),
        )
        end_key = (
            round(wp_end.transform.location.x, 2),
            round(wp_end.transform.location.y, 2),
            round(wp_end.transform.location.z, 2),
        )
        graph.add_edge(node_id_map[start_key], node_id_map[end_key])

    return graph, node_id_map


def euclidean_heuristic(a_id, b_id, graph):
    """欧几里得距离启发函数 — 可采纳"""
    x1, y1, _ = graph.nodes[a_id]
    x2, y2, _ = graph.nodes[b_id]
    return math.hypot(x2 - x1, y2 - y1)


def a_star_search(graph, start_id, goal_id, heuristic=euclidean_heuristic):
    """A* 搜索算法

    Args:
        graph: RoadGraph 实例
        start_id: 起始节点 ID
        goal_id: 目标节点 ID
        heuristic: 启发函数

    Returns:
        list: 从起点到目标的节点 ID 列表，若无路径则返回空列表
    """
    open_set = [(0.0, start_id)]
    came_from = {}

    g_score = {nid: float("inf") for nid in graph.nodes}
    g_score[start_id] = 0.0

    f_score = {nid: float("inf") for nid in graph.nodes}
    f_score[start_id] = heuristic(start_id, goal_id, graph)

    in_open = {start_id}
    visited_count = 0

    while open_set:
        _, current = heapq.heappop(open_set)
        in_open.discard(current)
        visited_count += 1

        if current == goal_id:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start_id)
            print(f"[A*] 搜索完成: 扩展 {visited_count} 节点, 路径长度 {len(path)}")
            return path[::-1]

        for neighbor, edge_cost in graph.get_neighbors(current):
            tentative_g = g_score[current] + edge_cost
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal_id, graph)
                if neighbor not in in_open:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    in_open.add(neighbor)

    print(f"[A*] 无路径可达 (扩展 {visited_count} 节点)")
    return []


def dijkstra_search(graph, start_id, goal_id):
    """Dijkstra 搜索 (用于对比)"""
    return a_star_search(graph, start_id, goal_id, heuristic=lambda a, b, g: 0.0)


def visualize_path(world, path, graph, color=carla.Color(0, 255, 0), life_time=30.0):
    """在 CARLA 世界中绘制路径"""
    if not path:
        print("[可视化] 路径为空，跳过绘制")
        return

    debug = world.debug
    for i in range(len(path) - 1):
        loc_start = graph.node_locations[path[i]]
        loc_end = graph.node_locations[path[i + 1]]
        loc_start.z += 0.5
        loc_end.z += 0.5
        debug.draw_line(
            loc_start,
            loc_end,
            thickness=0.3,
            color=color,
            life_time=life_time,
        )

    # 标记起点和终点
    start_loc = graph.node_locations[path[0]]
    goal_loc = graph.node_locations[path[-1]]
    start_loc.z += 1.0
    goal_loc.z += 1.0
    debug.draw_string(start_loc, "START", color=carla.Color(0, 255, 0), life_time=life_time)
    debug.draw_string(goal_loc, "GOAL", color=carla.Color(255, 0, 0), life_time=life_time)


def find_closest_node(graph, location):
    """找到距离给定 location 最近的图节点"""
    best_id = None
    best_dist = float("inf")
    for nid, loc in graph.node_locations.items():
        d = location.distance(loc)
        if d < best_dist:
            best_dist = d
            best_id = nid
    return best_id


def main():
    parser = argparse.ArgumentParser(description="A* 全局路径规划器 — CARLA")
    parser.add_argument("--host", type=str, default="localhost", help="CARLA 主机地址")
    parser.add_argument("--port", type=int, default=2000, help="CARLA 端口")
    parser.add_argument("--start-index", type=int, default=0, help="起点生成点索引")
    parser.add_argument("--goal-index", type=int, default=None, help="目标生成点索引")
    parser.add_argument("--dijkstra", action="store_true", help="使用 Dijkstra 替代 A*")
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    print("[全局规划器] 构建路网图...")
    graph, node_id_map = build_graph_from_topology(carla_map)
    print(f"[全局规划器] 节点数: {len(graph.nodes)}, 边数: {sum(len(v) for v in graph.edges.values())}")

    spawn_points = world.get_spawn_points()
    if not spawn_points:
        print("错误: 地图没有生成点")
        return

    start_loc = spawn_points[args.start_index].location
    goal_idx = args.goal_index if args.goal_index is not None else len(spawn_points) - 1
    goal_loc = spawn_points[goal_idx].location

    start_id = find_closest_node(graph, start_loc)
    goal_id = find_closest_node(graph, goal_loc)

    if start_id is None or goal_id is None:
        print("错误: 无法找到起点或目标对应的路网节点")
        return

    print(f"[全局规划器] 起点: ({start_loc.x:.1f}, {start_loc.y:.1f}) → 节点 {start_id}")
    print(f"[全局规划器] 目标: ({goal_loc.x:.1f}, {goal_loc.y:.1f}) → 节点 {goal_id}")

    if args.dijkstra:
        print("[全局规划器] 使用 Dijkstra 搜索...")
        path = dijkstra_search(graph, start_id, goal_id)
        color = carla.Color(0, 255, 255)
    else:
        print("[全局规划器] 使用 A* 搜索...")
        path = a_star_search(graph, start_id, goal_id)
        color = carla.Color(0, 255, 0)

    if path:
        total_length = 0.0
        for i in range(len(path) - 1):
            x1, y1, _ = graph.nodes[path[i]]
            x2, y2, _ = graph.nodes[path[i + 1]]
            total_length += math.hypot(x2 - x1, y2 - y1)
        print(f"[全局规划器] 路径总长度: {total_length:.1f} m")

        print("[全局规划器] 正在 CARLA 世界中绘制路径...")
        visualize_path(world, path, graph, color=color, life_time=60.0)
        print("[全局规划器] 完成! 路径在 CARLA 世界中可见 (绿色为 A*, 青色为 Dijkstra)")
    else:
        print("[全局规划器] 未找到可行路径")


if __name__ == "__main__":
    main()
