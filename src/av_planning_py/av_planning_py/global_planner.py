import math
import heapq

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header


class AStarPlanner:
    def __init__(self, use_diagonal=True, inflation_radius=1.0, grid_resolution=0.5):
        self.use_diagonal = use_diagonal
        self.inflation_radius = inflation_radius
        self.grid_resolution = grid_resolution
        self.occupancy_grid = None
        self.width = 0
        self.height = 0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.resolution = grid_resolution
        self.inflated_grid = None

    def set_map(self, occupancy_grid):
        self.occupancy_grid = occupancy_grid
        self.width = occupancy_grid.info.width
        self.height = occupancy_grid.info.height
        self.resolution = occupancy_grid.info.resolution
        self.origin_x = occupancy_grid.info.origin.position.x
        self.origin_y = occupancy_grid.info.origin.position.y
        self.inflated_grid = self._inflate_obstacles()

    def _inflate_obstacles(self):
        if self.occupancy_grid is None:
            return None
        grid = list(self.occupancy_grid.data)
        inflated = [0] * len(grid)
        inflation_cells = int(self.inflation_radius / self.resolution)
        if inflation_cells < 1:
            # 膨胀半径小于一个栅格时, 直接返回原始占用副本
            return [v if v > 50 else 0 for v in grid]

        for y in range(self.height):
            for x in range(self.width):
                idx = y * self.width + x
                if grid[idx] > 50:
                    for dy in range(-inflation_cells, inflation_cells + 1):
                        for dx in range(-inflation_cells, inflation_cells + 1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < self.width and 0 <= ny < self.height:
                                dist = math.hypot(dx, dy)
                                if dist <= inflation_cells:
                                    inflated[ny * self.width + nx] = 100
        return inflated

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = gx * self.resolution + self.origin_x + self.resolution / 2.0
        wy = gy * self.resolution + self.origin_y + self.resolution / 2.0
        return wx, wy

    def _get_neighbors(self, node):
        x, y = node
        neighbors = []
        if self.use_diagonal:
            directions = [(-1, -1), (-1, 0), (-1, 1),
                          (0, -1),           (0, 1),
                          (1, -1),  (1, 0),  (1, 1)]
        else:
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                idx = ny * self.width + nx
                if self.inflated_grid[idx] < 50:
                    cost = math.hypot(dx, dy)
                    neighbors.append(((nx, ny), cost))
        return neighbors

    def plan(self, start_world, goal_world):
        if self.occupancy_grid is None:
            return []

        sx, sy = self.world_to_grid(start_world[0], start_world[1])
        gx, gy = self.world_to_grid(goal_world[0], goal_world[1])

        if not (0 <= sx < self.width and 0 <= sy < self.height):
            return []
        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return []
        if self.inflated_grid[sy * self.width + sx] >= 50:
            return []
        if self.inflated_grid[gy * self.width + gx] >= 50:
            return []

        start = (sx, sy)
        goal = (gx, gy)

        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        f_score = {start: self._heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self._reconstruct_path(came_from, current)

            for neighbor, cost in self._get_neighbors(current):
                tentative_g = g_score[current] + cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def _heuristic(self, a, b):
        if self.use_diagonal:
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            return math.sqrt(2) * min(dx, dy) + abs(dx - dy)
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(current)
        path.reverse()
        return path


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner')

        self.declare_parameter('grid_resolution', 0.5)
        self.declare_parameter('inflation_radius', 1.0)
        self.declare_parameter('use_diagonal', True)

        self.planner = AStarPlanner(
            use_diagonal=self.get_parameter('use_diagonal').value,
            inflation_radius=self.get_parameter('inflation_radius').value,
            grid_resolution=self.get_parameter('grid_resolution').value,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 1)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.plan_pub = self.create_publisher(Path, '/plan', 10)
        self.marker_pub = self.create_publisher(Marker, '/plan_marker', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, '/plan_markers', 10)

        self.current_map = None
        self.get_logger().info('Global planner node started')

    def map_callback(self, msg):
        self.current_map = msg
        self.planner.set_map(msg)
        self.get_logger().info('Map received: %dx%d', msg.info.width, msg.info.height)

    def goal_callback(self, msg):
        if self.current_map is None:
            self.get_logger().warn('No map available, skipping planning')
            return

        start_x = self.current_map.info.origin.position.x + \
            self.current_map.info.width * self.current_map.info.resolution / 2.0
        start_y = self.current_map.info.origin.position.y + \
            self.current_map.info.height * self.current_map.info.resolution / 2.0

        goal = (msg.pose.position.x, msg.pose.position.y)
        start = (start_x, start_y)

        self.get_logger().info('Planning from (%.2f, %.2f) to (%.2f, %.2f)',
                               start[0], start[1], goal[0], goal[1])

        grid_path = self.planner.plan(start, goal)

        if not grid_path:
            self.get_logger().warn('No path found')
            return

        path_msg = Path()
        path_msg.header = Header(
            frame_id='map',
            stamp=self.get_clock().now().to_msg(),
        )

        for gx, gy in grid_path:
            wx, wy = self.planner.grid_to_world(gx, gy)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            path_msg.poses.append(pose)

        self.plan_pub.publish(path_msg)
        self._publish_markers(grid_path)
        self.get_logger().info('Published plan with %d waypoints', len(grid_path))

    def _publish_markers(self, grid_path):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'plan'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        for gx, gy in grid_path:
            wx, wy = self.planner.grid_to_world(gx, gy)
            p = Point()
            p.x = wx
            p.y = wy
            p.z = 0.0
            marker.points.append(p)

        self.marker_pub.publish(marker)

        markers = MarkerArray()
        for i, (gx, gy) in enumerate(grid_path):
            wx, wy = self.planner.grid_to_world(gx, gy)
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'plan_waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.0
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color.a = 1.0
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            markers.markers.append(m)

        self.marker_array_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

