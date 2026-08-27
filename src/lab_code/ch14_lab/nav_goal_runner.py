#!/usr/bin/env python3
"""
nav_goal_runner.py — Nav2 导航目标发送与监控脚本

功能:
  1. 设置机器人的初始位姿
  2. 发送连续导航目标点
  3. 监控导航过程（距离、状态、耗时）
  4. 支持多点连续导航

用法:
  ros2 run navigation_sim_demo_ros2 nav_goal_runner
  或
  python3 nav_goal_runner.py
"""

import math
import sys
import time
from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """偏航角转四元数 (简化版本, 仅考虑绕Z轴旋转)"""
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def planar_distance(x0: float, y0: float, x1: float, y1: float) -> float:
    """计算平面距离"""
    return math.hypot(x1 - x0, y1 - y0)


def build_pose(frame_id: str, x: float, y: float, yaw: float) -> PoseStamped:
    """构建位姿消息"""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    quat = yaw_to_quaternion(yaw)
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    return pose


class OdomWatcher(Node):
    """里程计监控节点"""

    def __init__(self) -> None:
        super().__init__("nav_goal_odom_watcher")
        self.latest: Optional[Odometry] = None
        self.subscription = self.create_subscription(
            Odometry, "/odom", self.callback, 10)

    def callback(self, message: Odometry) -> None:
        self.latest = message

    def get_xy(self) -> Optional[tuple[float, float]]:
        if self.latest is None:
            return None
        return (
            float(self.latest.pose.pose.position.x),
            float(self.latest.pose.pose.position.y),
        )


def stamp_pose(node: Node, pose: PoseStamped) -> PoseStamped:
    """为位姿添加时间戳"""
    pose.header.stamp = node.get_clock().now().to_msg()
    return pose


def navigate_to_goal(navigator, odom_watcher, executor,
                     x: float, y: float, yaw: float,
                     timeout: float = 15.0) -> bool:
    """
    发送导航目标并等待完成

    返回: True 表示导航成功, False 表示失败/超时
    """
    start_time = time.time()
    start_pos = odom_watcher.get_xy()

    goal_pose = stamp_pose(navigator, build_pose("map", x, y, yaw))
    goal = NavigateToPose.Goal()
    goal.pose = goal_pose

    goal_future = navigator.send_goal_async(goal, feedback_callback=None)
    rclpy.spin_until_future_complete(navigator, goal_future, timeout_sec=3.0)
    goal_handle = goal_future.result()

    if goal_handle is None or not goal_handle.accepted:
        print(f"  目标 ({x:.1f}, {y:.1f}) 被拒绝!")
        return False

    print(f"  导航到 ({x:.1f}, {y:.1f}, {yaw:.1f})...")

    # 等待导航完成或超时
    while time.time() - start_time < timeout:
        executor.spin_once(timeout_sec=0.1)
        status = navigator.get_result()
        if status is not None:
            if status == NavigateToPose.GoalStatus.STATUS_SUCCEEDED:
                elapsed = time.time() - start_time
                end_pos = odom_watcher.get_xy()
                if end_pos and start_pos:
                    dist = planar_distance(start_pos[0], start_pos[1], end_pos[0], end_pos[1])
                    print(f"  到达目标! 耗时: {elapsed:.1f}s, 行驶: {dist:.2f}m")
                return True
            else:
                print(f"  导航失败, 状态码: {status}")
                return False

        # 打印反馈
        feedback = navigator.get_feedback()
        if feedback and hasattr(feedback, 'distance_remaining'):
            remaining = feedback.distance_remaining
            if remaining is not None:
                print(f"    距目标: {remaining:.2f}m", end="\r")
                sys.stdout.flush()

    print("")
    print(f"  导航超时 ({timeout}s)")
    return False


def main() -> None:
    # 定义导航目标点序列 (x, y, yaw)
    waypoints = [
        (3.0, -1.0, -2.0),    # 目标1: 前方偏右
        (2.0, 1.5, 1.57),     # 目标2: 左侧
        (5.0, 0.5, 0.0),      # 目标3: 前方
        (1.0, -0.5, 3.14),    # 目标4: 返回起点附近
    ]

    rclpy.init()
    navigator = BasicNavigator()
    odom_watcher = OdomWatcher()
    use_sim_time = bool(odom_watcher.get_parameter("use_sim_time").value)
    navigator.set_parameters([Parameter("use_sim_time", value=use_sim_time)])
    navigate_action = ActionClient(navigator, NavigateToPose, "navigate_to_pose")
    executor = SingleThreadedExecutor()
    executor.add_node(navigator)
    executor.add_node(odom_watcher)

    try:
        # 设置初始位姿
        print("==============================================")
        print("  Nav2 导航目标发送脚本")
        print("==============================================")
        start_pose = stamp_pose(navigator, build_pose("map", 5.0, 0.0, -2.0))
        navigator.setInitialPose(start_pose)
        print(f"初始位姿: (5.0, 0.0, -2.0)")

        # 等待 Nav2 就绪
        print("等待 Nav2 就绪...")
        navigator.waitUntilNav2Active()
        if not navigate_action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("NavigateToPose action server 未就绪")
        print("Nav2 就绪!")

        # 按顺序导航到各个目标点
        success_count = 0
        for i, (x, y, yaw) in enumerate(waypoints, 1):
            print(f"\n目标 {i}/{len(waypoints)}:")
            if navigate_to_goal(navigator, odom_watcher, executor, x, y, yaw):
                success_count += 1
            else:
                print(f"  跳过剩余目标点...")
                break

        # 打印汇总
        print("\n==============================================")
        print(f"  导航完成: {success_count}/{len(waypoints)} 个目标成功到达")
        print("==============================================")

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        executor.remove_node(navigator)
        executor.remove_node(odom_watcher)
        odom_watcher.destroy_node()
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
