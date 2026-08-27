"""waypoint_pub.py — CARLA Waypoint 路径生成与 ROS2 发布节点

从起点到目标点沿车道中心线生成连续 Waypoint 路径，
发布为 nav_msgs/Path 消息供下游控制节点使用。

ROS 2 用法:
    python3 waypoint_pub.py

独立用法 (不依赖 ROS2):
    python waypoint_pub.py --standalone

依赖:
    carla
    rclpy, geometry_msgs, nav_msgs (ROS2 模式)
"""

import math
import sys

try:
    import carla
except ImportError:
    raise ImportError("carla 模块未找到")


class WaypointPathGenerator:
    """基于 CARLA Waypoint API 的路径生成器"""

    def __init__(self, carla_map):
        self.map = carla_map

    def generate_path(self, start_location, goal_location=None, step=2.0, max_steps=1000):
        """生成从起点开始的连续 Waypoint 路径

        Args:
            start_location: carla.Location，起点位置
            goal_location: carla.Location 或 None，目标位置
            step: 相邻 waypoint 间距 (米)
            max_steps: 最大步数

        Returns:
            list[carla.Waypoint]: 路径 waypoint 列表
        """
        waypoint = self.map.get_waypoint(start_location)
        if waypoint is None:
            print("[WaypointGen] 无法在起点位置找到对应 waypoint")
            return []

        path = [waypoint]
        for _ in range(max_steps):
            next_wps = waypoint.next(step)
            if not next_wps:
                break

            # 多岔路时选择朝向变化最小的后继
            next_wp = min(
                next_wps,
                key=lambda wp: abs(
                    wp.transform.rotation.yaw - waypoint.transform.rotation.yaw
                ),
            )
            path.append(next_wp)
            waypoint = next_wp

            # 若设置了目标且已接近，提前终止
            if goal_location is not None:
                dist = waypoint.transform.location.distance(goal_location)
                if dist < step:
                    break

        return path

    def generate_path_to_lane(self, start_location, target_lane_type="driving", step=2.0):
        """生成路径并确保始终在目标类型车道"""
        wp = self.map.get_waypoint(start_location)
        if wp is None:
            return []

        path = []
        for _ in range(1000):
            if wp is None:
                break
            if wp.lane_type != carla.LaneType.Driving:
                next_wp = wp.next(step)
                wp = next_wp[0] if next_wp else None
                continue
            path.append(wp)
            next_wps = wp.next(step)
            if not next_wps:
                break
            wp = min(
                next_wps,
                key=lambda w: abs(w.transform.rotation.yaw - wp.transform.rotation.yaw),
            )

        return path

    @staticmethod
    def path_to_waypoint_locations(path):
        """提取 waypoint 的位置列表"""
        return [wp.transform.location for wp in path]

    @staticmethod
    def export_to_kml(path, filename="route.kml"):
        """将路径导出为 KML 格式"""
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
            f"{wp.transform.location.x},{wp.transform.location.y},0" for wp in path
        )
        with open(filename, "w", encoding="utf-8") as f:
            f.write(kml_header + coords + kml_footer)
        print(f"[KML] 路径已导出: {filename} (共 {len(path)} 个 waypoint)")


try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class WaypointPublisherNode(Node):
    """ROS2 节点：定期发布 Waypoint 路径"""

    def __init__(self):
        super().__init__("waypoint_publisher")

        self.declare_parameter("host", "localhost")
        self.declare_parameter("port", 2000)
        self.declare_parameter("step", 2.0)
        self.declare_parameter("publish_rate", 0.5)
        self.declare_parameter("frame_id", "map")

        host = self.get_parameter("host").value
        port = self.get_parameter("port").value
        step = self.get_parameter("step").value
        rate = self.get_parameter("publish_rate").value
        self.frame_id = self.get_parameter("frame_id").value

        self.publisher = self.create_publisher(Path, "/planned_path", 10)

        client = carla.Client(host, port)
        client.set_timeout(10.0)
        world = client.get_world()
        self.generator = WaypointPathGenerator(world.get_map())

        spawn_points = world.get_spawn_points()
        if not spawn_points:
            self.get_logger().error("地图没有生成点!")
            sys.exit(1)

        self.start_spawn = spawn_points[0]
        self.goal_spawn = spawn_points[-1]

        self.timer = self.create_timer(1.0 / rate, self.publish_path)
        self.get_logger().info(f"Waypoint 发布器已启动: {step}m 步长, {rate}Hz")
        self.get_logger().info(f"  起点: ({self.start_spawn.location.x:.1f}, {self.start_spawn.location.y:.1f})")
        self.get_logger().info(f"  目标: ({self.goal_spawn.location.x:.1f}, {self.goal_spawn.location.y:.1f})")

    def publish_path(self):
        path_wps = self.generator.generate_path(
            self.start_spawn.location,
            goal_location=self.goal_spawn.location,
            step=2.0,
        )

        ros_path = Path()
        ros_path.header.stamp = self.get_clock().now().to_msg()
        ros_path.header.frame_id = self.frame_id

        for wp in path_wps:
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = wp.transform.location.x
            pose.pose.position.y = wp.transform.location.y
            pose.pose.position.z = wp.transform.location.z
            pose.pose.orientation.x = wp.transform.rotation.x
            pose.pose.orientation.y = wp.transform.rotation.y
            pose.pose.orientation.z = wp.transform.rotation.z
            pose.pose.orientation.w = wp.transform.rotation.w
            ros_path.poses.append(pose)

        self.publisher.publish(ros_path)
        self.get_logger().info(f"已发布路径: {len(path_wps)} 个 waypoint")


def run_standalone():
    """独立模式（不依赖 ROS2）"""
    import time

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    generator = WaypointPathGenerator(carla_map)
    spawn_points = world.get_spawn_points()

    if not spawn_points:
        print("错误: 地图没有生成点")
        return

    start = spawn_points[0].location
    goal = spawn_points[-1].location

    print(f"[Standalone] 起点: ({start.x:.1f}, {start.y:.1f})")
    print(f"[Standalone] 目标: ({goal.x:.1f}, {goal.y:.1f})")

    path = generator.generate_path(start, goal_location=goal, step=2.0)
    print(f"[Standalone] 路径生成完成: {len(path)} 个 waypoint")

    # 在 CARLA 世界中可视化
    debug = world.debug
    for i in range(len(path) - 1):
        loc_a = path[i].transform.location
        loc_b = path[i + 1].transform.location
        loc_a.z += 0.3
        loc_b.z += 0.3
        debug.draw_line(
            loc_a, loc_b, thickness=0.2, color=carla.Color(0, 255, 0), life_time=30.0
        )

    # 导出 KML
    generator.export_to_kml(path)

    print("[Standalone] 路径可视化完成，请查看 CARLA 窗口")


def main():
    if "--standalone" in sys.argv:
        run_standalone()
    elif ROS2_AVAILABLE:
        rclpy.init()
        node = WaypointPublisherNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("ROS2 不可用，请使用 --standalone 标志运行独立模式")
        sys.exit(1)


if __name__ == "__main__":
    main()
