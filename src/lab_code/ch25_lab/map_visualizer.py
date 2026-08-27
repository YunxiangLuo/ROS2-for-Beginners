"""map_visualizer.py — CARLA 地图 OpenDRIVE 可视化工具

从 CARLA 模拟器获取地图 OpenDRIVE 数据，进行解析和可视化：
  1. 导出 OpenDRIVE XML 文件
  2. 解析 XML 提取道路/车道信息
  3. 使用 matplotlib 绘制路网拓扑图
  4. 可视化 Waypoint 车道中心线

用法:
    python map_visualizer.py [--host HOST] [--port PORT] [--output DIR]

依赖:
    carla, matplotlib, numpy, xml.etree
"""

import argparse
import os
import xml.etree.ElementTree as ET
from collections import defaultdict

import numpy as np

try:
    import carla
except ImportError:
    raise ImportError("carla 模块未找到。请确保已安装 CARLA Python API。")

try:
    import matplotlib.pyplot as plt
except ImportError:
    raise ImportError("matplotlib 未安装。请执行: pip install matplotlib")


class OpenDRIVEParser:
    """OpenDRIVE XML 解析器"""

    def __init__(self, xml_string):
        self.root = ET.fromstring(xml_string)
        self.roads = []

    def parse_roads(self):
        """解析所有道路信息"""
        road_data = []
        for road_elem in self.root.findall(".//road"):
            road_id = road_elem.get("id")
            road_name = road_elem.get("name", "")
            road_length = float(road_elem.get("length", 0))

            # 解析车道
            lane_section = road_elem.find(".//laneSection")
            n_left = 0
            n_right = 0
            if lane_section is not None:
                left = lane_section.find("left")
                right = lane_section.find("right")
                n_left = len(left.findall("lane")) if left is not None else 0
                n_right = len(right.findall("lane")) if right is not None else 0

            # 解析几何形状
            geometries = []
            for geom in road_elem.findall(".//planView/geometry"):
                geom_type = list(geom)[0].tag if list(geom) else "unknown"
                geometries.append({
                    "s": float(geom.get("s", 0)),
                    "x": float(geom.get("x", 0)),
                    "y": float(geom.get("y", 0)),
                    "hdg": float(geom.get("hdg", 0)),
                    "length": float(geom.get("length", 0)),
                    "type": geom_type,
                })

            road_data.append({
                "id": road_id,
                "name": road_name,
                "length": road_length,
                "left_lanes": n_left,
                "right_lanes": n_right,
                "geometries": geometries,
            })

        self.roads = road_data
        return road_data

    def print_road_summary(self):
        """打印道路概要信息"""
        if not self.roads:
            self.parse_roads()

        print(f"{'='*60}")
        print(f"  OpenDRIVE 道路统计")
        print(f"{'='*60}")
        print(f"  道路总数: {len(self.roads)}")
        print(f"{'='*60}")

        for rd in self.roads:
            n_lanes = rd["left_lanes"] + rd["right_lanes"]
            print(f"  道路 {rd['id']:>4s}: {rd['name']:<20s} "
                  f"长度={rd['length']:>7.1f}m  车道数={n_lanes}")

        total_length = sum(rd["length"] for rd in self.roads)
        total_lanes = sum(rd["left_lanes"] + rd["right_lanes"] for rd in self.roads)
        print(f"{'='*60}")
        print(f"  总道路长度: {total_length:.1f} m")
        print(f"  总车道数:   {total_lanes}")
        print(f"{'='*60}")

    def get_road_centerline_points(self, samples_per_meter=0.5):
        """估算每条道路的中心线点"""
        road_lines = {}
        for rd in self.roads:
            points = []
            for geom in rd["geometries"]:
                x0, y0 = geom["x"], geom["y"]
                hdg = geom["hdg"]
                length = geom["length"]
                n_pts = max(2, int(length * samples_per_meter))
                for i in range(n_pts):
                    s = (i / n_pts) * length
                    x = x0 + s * np.cos(hdg)
                    y = y0 + s * np.sin(hdg)
                    points.append((x, y))
            road_lines[rd["id"]] = points
        return road_lines


class MapVisualizer:
    """CARLA 地图可视化器"""

    def __init__(self, host="localhost", port=2000):
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.carla_map = self.world.get_map()
        self.opendrive_str = self.carla_map.to_opendrive()
        self.parser = OpenDRIVEParser(self.opendrive_str)

    def export_opendrive(self, output_dir="."):
        """导出 OpenDRIVE XML 到文件"""
        os.makedirs(output_dir, exist_ok=True)
        filepath = os.path.join(output_dir, "carla_map.xodr")
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(self.opendrive_str)
        print(f"[导出] OpenDRIVE 已保存: {filepath} ({len(self.opendrive_str)} 字符)")
        return filepath

    def plot_topology(self, output_dir=".", filename="road_topology.png"):
        """绘制道路拓扑图"""
        topology = self.carla_map.get_topology()
        print(f"[绘图] 道路拓扑边数: {len(topology)}")

        plt.figure(figsize=(14, 10))
        for edge in topology:
            wp_start, wp_end = edge
            loc_s = wp_start.transform.location
            loc_e = wp_end.transform.location
            plt.plot([loc_s.x, loc_e.x], [loc_s.y, loc_e.y],
                     "b-", alpha=0.4, linewidth=0.6)
            plt.plot(loc_s.x, loc_s.y, "g.", markersize=1.5)
            plt.plot(loc_e.x, loc_e.y, "r.", markersize=1.5)

        plt.axis("equal")
        plt.title("CARLA 道路拓扑网络", fontsize=14)
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.grid(True, alpha=0.3)

        filepath = os.path.join(output_dir, filename)
        plt.savefig(filepath, dpi=150, bbox_inches="tight")
        plt.close()
        print(f"[绘图] 拓扑图已保存: {filepath}")
        return filepath

    def plot_centerlines(self, output_dir=".", filename="road_centerlines.png"):
        """绘制道路中心线"""
        self.parser.parse_roads()
        road_lines = self.parser.get_road_centerline_points(samples_per_meter=0.5)

        plt.figure(figsize=(14, 10))
        for road_id, points in road_lines.items():
            if not points:
                continue
            xs, ys = zip(*points)
            plt.plot(xs, ys, "-", linewidth=1.0, alpha=0.7, label=f"Road {road_id}")

        plt.axis("equal")
        plt.title("CARLA 道路中心线", fontsize=14)
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.grid(True, alpha=0.3)

        filepath = os.path.join(output_dir, filename)
        plt.savefig(filepath, dpi=150, bbox_inches="tight")
        plt.close()
        print(f"[绘图] 中心线图已保存: {filepath}")
        return filepath

    def plot_lane_waypoints(self, output_dir=".", filename="lane_waypoints.png"):
        """可视化车道 waypoint"""
        spawn_points = self.world.get_spawn_points()
        if not spawn_points:
            print("[绘图] 没有生成点")
            return

        waypoints = []
        for sp in spawn_points[:50]:
            wp = self.carla_map.get_waypoint(sp.location)
            if wp:
                waypoints.append(wp)

        plt.figure(figsize=(14, 10))
        for wp in waypoints:
            loc = wp.transform.location
            yaw = wp.transform.rotation.yaw
            dx = 2.0 * np.cos(np.radians(yaw))
            dy = 2.0 * np.sin(np.radians(yaw))

            plt.plot(loc.x, loc.y, "b.", markersize=2)
            plt.arrow(loc.x, loc.y, dx, dy,
                      head_width=0.5, head_length=0.8,
                      fc="green", ec="green", alpha=0.6)

            # 显示车道 ID
            plt.annotate(str(wp.lane_id), (loc.x, loc.y),
                         fontsize=5, alpha=0.6)

        plt.axis("equal")
        plt.title("CARLA 车道 Waypoint (箭头表示朝向)", fontsize=14)
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.grid(True, alpha=0.3)

        filepath = os.path.join(output_dir, filename)
        plt.savefig(filepath, dpi=150, bbox_inches="tight")
        plt.close()
        print(f"[绘图] Waypoint 图已保存: {filepath}")
        return filepath

    def plot_lanelet_style(self, output_dir=".", filename="lanelet_style.png"):
        """以 Lanelet 风格可视化车道"""
        topology = self.carla_map.get_topology()
        plt.figure(figsize=(14, 10))

        for i, edge in enumerate(topology):
            wp_start, wp_end = edge
            loc_s = wp_start.transform.location
            loc_e = wp_end.transform.location

            # 左右偏移模拟车道宽度
            yaw = np.arctan2(loc_e.y - loc_s.y, loc_e.x - loc_s.x)
            offset = 1.75

            left_s = (loc_s.x + offset * np.cos(yaw + np.pi / 2),
                      loc_s.y + offset * np.sin(yaw + np.pi / 2))
            right_s = (loc_s.x - offset * np.cos(yaw + np.pi / 2),
                       loc_s.y - offset * np.sin(yaw + np.pi / 2))
            left_e = (loc_e.x + offset * np.cos(yaw + np.pi / 2),
                      loc_e.y + offset * np.sin(yaw + np.pi / 2))
            right_e = (loc_e.x - offset * np.cos(yaw + np.pi / 2),
                       loc_e.y - offset * np.sin(yaw + np.pi / 2))

            plt.plot([left_s[0], left_e[0]], [left_s[1], left_e[1]],
                     "k-", linewidth=0.8, alpha=0.5)
            plt.plot([right_s[0], right_e[0]], [right_s[1], right_e[1]],
                     "k-", linewidth=0.8, alpha=0.5)

            if i % 3 == 0:
                plt.fill([left_s[0], left_e[0], right_e[0], right_s[0]],
                         [left_s[1], left_e[1], right_e[1], right_s[1]],
                         alpha=0.1, color="lightblue")

        plt.axis("equal")
        plt.title("Lanelet 风格车道可视化", fontsize=14)
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")

        filepath = os.path.join(output_dir, filename)
        plt.savefig(filepath, dpi=150, bbox_inches="tight")
        plt.close()
        print(f"[绘图] Lanelet 风格图已保存: {filepath}")
        return filepath

    def print_debug_info(self):
        """打印地图调试信息"""
        print(f"\n{'='*60}")
        print("  CARLA 地图调试信息")
        print(f"{'='*60}")
        print(f"  地图名称: {self.carla_map.name}")
        print(f"  生成点数: {len(self.world.get_spawn_points())}")
        print(f"  OpenDRIVE 长度: {len(self.opendrive_str)} 字符")
        print()

        self.parser.parse_roads()
        self.parser.print_road_summary()

        # 交叉口信息
        topology = self.carla_map.get_topology()
        junctions = set()
        for wp_start, _ in topology:
            if wp_start.is_junction:
                junctions.add(wp_start.junction_id)
        print(f"  交叉口 (Junction) 数量: {len(junctions)}")
        print(f"{'='*60}\n")


def main():
    parser = argparse.ArgumentParser(description="CARLA 地图 OpenDRIVE 可视化工具")
    parser.add_argument("--host", type=str, default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--output", type=str, default="output", help="输出目录")
    parser.add_argument("--no-export", action="store_true", help="不导出 OpenDRIVE")
    parser.add_argument("--no-plot", action="store_true", help="不生成图片")
    args = parser.parse_args()

    print("[MapVisualizer] 初始化...")
    visualizer = MapVisualizer(host=args.host, port=args.port)
    visualizer.print_debug_info()

    output_dir = os.path.abspath(args.output)

    if not args.no_export:
        visualizer.export_opendrive(output_dir)

    if not args.no_plot:
        print("\n[MapVisualizer] 生成可视化图片...")
        visualizer.plot_topology(output_dir)
        visualizer.plot_centerlines(output_dir)
        visualizer.plot_lane_waypoints(output_dir)
        visualizer.plot_lanelet_style(output_dir)
        print(f"\n[MapVisualizer] 所有图片已保存至: {output_dir}")
    else:
        print("[MapVisualizer] 跳过图片生成")

    print("[MapVisualizer] 完成!")


if __name__ == "__main__":
    main()
