import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def planar_distance(x0: float, y0: float, x1: float, y1: float) -> float:
    return math.hypot(x1 - x0, y1 - y0)


def build_pose(frame_id: str, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    quat = yaw_to_quaternion(yaw)
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    return pose


class NavigationGoalRunner(Node):
    def __init__(self) -> None:
        super().__init__("nav_goal_runner")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("goal_x", 1.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("server_timeout_sec", 30.0)
        self.declare_parameter("motion_timeout_sec", 20.0)
        self.latest = None
        self.subscription = self.create_subscription(Odometry, "/odom", self.callback, 10)
        self.navigate_action = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def callback(self, message: Odometry) -> None:
        self.latest = message


def stamp_pose(node: Node, pose: PoseStamped) -> PoseStamped:
    pose.header.stamp = node.get_clock().now().to_msg()
    return pose


def main() -> None:
    rclpy.init()
    node = NavigationGoalRunner()

    try:
        goal_pose = stamp_pose(
            node,
            build_pose(
                "map",
                float(node.get_parameter("goal_x").value),
                float(node.get_parameter("goal_y").value),
                float(node.get_parameter("goal_yaw").value),
            ),
        )
        server_timeout = float(node.get_parameter("server_timeout_sec").value)
        if not node.navigate_action.wait_for_server(timeout_sec=server_timeout):
            raise RuntimeError("NavigateToPose action server did not become ready")

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        goal_future = node.navigate_action.send_goal_async(goal, feedback_callback=None)
        rclpy.spin_until_future_complete(node, goal_future, timeout_sec=10.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("NavigateToPose goal was rejected")

        start_time = time.time()
        start_odom_position = None
        motion_timeout = float(node.get_parameter("motion_timeout_sec").value)
        while time.time() - start_time < motion_timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.latest is None:
                continue
            latest_x = float(node.latest.pose.pose.position.x)
            latest_y = float(node.latest.pose.pose.position.y)
            if start_odom_position is None:
                start_odom_position = (latest_x, latest_y)
                continue
            if planar_distance(start_odom_position[0], start_odom_position[1], latest_x, latest_y) > 0.05:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=3.0)
                print("navigation-motion-detected")
                return

        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=3.0)
        raise RuntimeError("Robot odometry did not change after sending NavigateToPose goal")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
