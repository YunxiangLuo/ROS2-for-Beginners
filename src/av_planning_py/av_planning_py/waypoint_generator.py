import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from av_carla_interfaces.msg import WaypointArray, Waypoint


class WaypointGeneratorNode(Node):
    def __init__(self):
        super().__init__('waypoint_generator')

        self.declare_parameter('waypoint_spacing', 2.0)
        self.declare_parameter('default_speed', 10.0)
        self.declare_parameter('max_speed', 20.0)

        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.default_speed = self.get_parameter('default_speed').value
        self.max_speed = self.get_parameter('max_speed').value

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.waypoints_pub = self.create_publisher(WaypointArray, '/waypoints', 10)

        self.route_points = []

        self.get_logger().info('Waypoint generator started')

    def goal_callback(self, msg):
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        self.get_logger().info('Generating waypoints to (%.2f, %.2f)', goal_x, goal_y)

        self.route_points = self._generate_route(goal_x, goal_y)

        waypoints = self._interpolate_waypoints(self.route_points)
        waypoint_msg = self._build_waypoint_array(waypoints)

        self.waypoints_pub.publish(waypoint_msg)
        self.get_logger().info('Published %d waypoints', len(waypoints))

    def _generate_route(self, goal_x, goal_y):
        points = [(0.0, 0.0)]
        segments = max(3, int(math.hypot(goal_x, goal_y) / self.waypoint_spacing))
        for i in range(1, segments + 1):
            t = i / segments
            x = goal_x * t
            y = goal_y * t
            points.append((x, y))
        if points[-1] != (goal_x, goal_y):
            points.append((goal_x, goal_y))
        return points

    def _interpolate_waypoints(self, route_points):
        waypoints = []
        for i, (x, y) in enumerate(route_points):
            speed = self._compute_speed(x, y, route_points, i)
            wp = Waypoint()
            wp.x = x
            wp.y = y
            wp.z = 0.0
            wp.speed = speed
            wp.lane_id = 0
            wp.road_id = str(i)
            waypoints.append(wp)
        return waypoints

    def _compute_speed(self, x, y, route, idx):
        return float(self.default_speed)

    def _build_waypoint_array(self, waypoints):
        msg = WaypointArray()
        msg.header = Header(
            frame_id='map',
            stamp=self.get_clock().now().to_msg(),
        )
        msg.waypoints = waypoints
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = WaypointGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
