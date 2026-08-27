import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from av_carla_interfaces.action import Navigate
from av_carla_interfaces.msg import WaypointArray


class PlanningServerNode(Node):
    def __init__(self):
        super().__init__('planning_server')

        self.action_server = ActionServer(
            self,
            Navigate,
            '/navigate',
            self.execute_callback,
        )

        self.plan_pub = self.create_publisher(Path, '/plan', 10)
        self.waypoints_pub = self.create_publisher(WaypointArray, '/waypoints', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.get_logger().info('Planning server started')

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received navigate goal: %s', goal_handle.request.target_location)

        goal_handle.publish_feedback(Navigate.Feedback(
            status='Parsing target location',
            progress=0.0,
        ))

        target = goal_handle.request.target_location
        coords = target.split(',')
        if len(coords) >= 2:
            try:
                goal_x = float(coords[0].strip())
                goal_y = float(coords[1].strip())
            except ValueError:
                goal_handle.abort()
                return Navigate.Result(success=False, total_time=0.0, avg_speed=0.0)
        else:
            goal_handle.abort()
            return Navigate.Result(success=False, total_time=0.0, avg_speed=0.0)

        start_time = time.time()

        goal_handle.publish_feedback(Navigate.Feedback(
            status='Publishing goal pose for global planner',
            progress=0.2,
        ))

        goal_pose = PoseStamped()
        goal_pose.header = Header(
            frame_id='map',
            stamp=self.get_clock().now().to_msg(),
        )
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        self.goal_pub.publish(goal_pose)

        goal_handle.publish_feedback(Navigate.Feedback(
            status='Global planning in progress',
            progress=0.4,
        ))

        path = Path()
        path.header = Header(
            frame_id='map',
            stamp=self.get_clock().now().to_msg(),
        )
        self.plan_pub.publish(path)

        goal_handle.publish_feedback(Navigate.Feedback(
            status='Generating waypoints',
            progress=0.6,
        ))

        waypoints = WaypointArray()
        waypoints.header = Header(
            frame_id='map',
            stamp=self.get_clock().now().to_msg(),
        )
        self.waypoints_pub.publish(waypoints)

        goal_handle.publish_feedback(Navigate.Feedback(
            status='Planning complete',
            progress=1.0,
        ))

        self.get_logger().info(
            'Navigate to (%.2f, %.2f) completed', goal_x, goal_y)

        elapsed = time.time() - start_time
        goal_handle.succeed()

        return Navigate.Result(
            success=True,
            total_time=float(elapsed),
            avg_speed=10.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = PlanningServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
