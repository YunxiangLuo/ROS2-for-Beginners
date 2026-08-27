#!/usr/bin/env python3
"""
slam_map_runner.py — SLAM 建图监控与机器人自动探索脚本

功能:
  1. 自动驱动机器人探索环境
  2. 监控地图构建进度
  3. 计算地图探索率和建图质量
  4. 达到条件后自动停止

用法:
  ros2 run slam_sim_demo_ros2 slam_map_runner
  或
  python3 slam_map_runner.py
"""

import math
import time
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


def count_known_cells(cells: list[int]) -> int:
    """计算已知区域栅格数（非未知区域）"""
    return sum(1 for value in cells if value >= 0)


def count_occupied_cells(cells: list[int]) -> int:
    """计算占据栅格数"""
    return sum(1 for value in cells if value == 100)


def count_free_cells(cells: list[int]) -> int:
    """计算空闲栅格数"""
    return sum(1 for value in cells if value == 0)


def planar_distance(start: tuple[float, float], end: tuple[float, float]) -> float:
    """计算平面距离"""
    return math.hypot(end[0] - start[0], end[1] - start[1])


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """四元数转偏航角"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def command_for_elapsed(elapsed_sec: float) -> tuple[float, float]:
    """
    根据经过时间生成机器人运动指令
    生成周期性运动：左转→前进→右转→前进
    """
    cycle_time = elapsed_sec % 8.0
    if cycle_time < 2.0:
        return (0.0, 0.6)       # 左转
    if cycle_time < 4.5:
        return (0.25, 0.0)      # 前进
    if cycle_time < 6.5:
        return (0.0, -0.6)      # 右转
    return (0.25, 0.0)          # 前进


class SlamMapCheckNode(Node):
    """SLAM 建图监控节点"""

    def __init__(self) -> None:
        super().__init__("slam_map_runner")
        self.command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_subscription = self.create_subscription(
            Odometry, "/odom", self.handle_odom, 10)
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/map", self.handle_map, 10)

        self.start_pose: Optional[tuple[float, float]] = None
        self.latest_pose: Optional[tuple[float, float]] = None
        self.latest_yaw: float = 0.0
        self.first_map_known_cells: Optional[int] = None
        self.first_map_info: Optional[dict] = None
        self.max_known_cells: int = 0
        self.latest_map_data: Optional[OccupancyGrid] = None
        self.map_updates: int = 0
        self.last_map_stamp: Optional[tuple[int, int]] = None

        self.get_logger().info("SLAM 建图监控节点已启动")

    def handle_odom(self, message: Odometry) -> None:
        """处理里程计数据"""
        pose = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )
        self.latest_yaw = yaw_from_quaternion(
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        )
        if self.start_pose is None:
            self.start_pose = pose
            self.get_logger().info(f"起始位置: ({pose[0]:.2f}, {pose[1]:.2f})")
        self.latest_pose = pose

    def handle_map(self, message: OccupancyGrid) -> None:
        """处理地图数据"""
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        if stamp != self.last_map_stamp:
            self.map_updates += 1
            self.last_map_stamp = stamp
            self.latest_map_data = message

            if self.first_map_info is None:
                self.first_map_info = {
                    "width": message.info.width,
                    "height": message.info.height,
                    "resolution": message.info.resolution,
                }

        cells = list(message.data)
        known_cells = count_known_cells(cells)
        occupied = count_occupied_cells(cells)
        free = count_free_cells(cells)
        total = known_cells

        if self.first_map_known_cells is None:
            self.first_map_known_cells = known_cells
            self.get_logger().info(f"首次接收到地图: "
                f"{message.info.width}x{message.info.height} @ {message.info.resolution:.3f}m/px")

        self.max_known_cells = max(self.max_known_cells, known_cells)

        # 每5次更新打印一次地图状态
        if self.map_updates % 5 == 0 and total > 0:
            exploration_ratio = known_cells / (message.info.width * message.info.height) * 100
            occupied_ratio = occupied / total * 100
            self.get_logger().info(
                f"[更新 #{self.map_updates}] "
                f"已知: {known_cells}, 占据: {occupied}({occupied_ratio:.1f}%), "
                f"空闲: {free}, 探索率: {exploration_ratio:.1f}%"
            )

    def ready(self) -> bool:
        """检查节点是否就绪"""
        return (self.latest_pose is not None
                and self.first_map_known_cells is not None)

    def odom_distance(self) -> float:
        """计算行驶距离"""
        if self.start_pose is None or self.latest_pose is None:
            return 0.0
        return planar_distance(self.start_pose, self.latest_pose)

    def exploration_ratio(self) -> float:
        """计算探索率"""
        if self.latest_map_data is None or self.first_map_info is None:
            return 0.0
        total_cells = self.first_map_info["width"] * self.first_map_info["height"]
        known = count_known_cells(list(self.latest_map_data.data))
        return known / total_cells * 100

    def publish_command(self, linear_x: float, angular_z: float) -> None:
        """发布速度指令"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.command_publisher.publish(twist)

    def publish_stop(self) -> None:
        """停止机器人"""
        self.publish_command(0.0, 0.0)
        self.get_logger().info("机器人已停止")

    def succeeded(self) -> bool:
        """判断建图是否完成"""
        if self.first_map_known_cells is None:
            return False
        return (
            self.map_updates >= 2
            and self.odom_distance() > 0.2
            and self.max_known_cells - self.first_map_known_cells > 100
        )


def main() -> None:
    rclpy.init()
    node = SlamMapCheckNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    overall_start = time.time()
    motion_start: Optional[float] = None
    timeout = 25.0

    try:
        while time.time() - overall_start < timeout:
            executor.spin_once(timeout_sec=0.1)
            if not node.ready():
                continue

            if motion_start is None:
                motion_start = time.time()
                node.get_logger().info("开始驱动机器人探索...")

            elapsed = time.time() - motion_start
            linear_x, angular_z = command_for_elapsed(elapsed)
            node.publish_command(linear_x, angular_z)

            if node.succeeded():
                node.publish_stop()
                node.get_logger().info(f"建图完成！探索率: {node.exploration_ratio():.1f}%")
                print("slam-map-updated")
                return

        node.publish_stop()
        current_ratio = node.exploration_ratio()
        raise RuntimeError(
            f"slam_toolbox 未能在 {timeout}s 内完成建图 (探索率: {current_ratio:.1f}%)"
        )
    except KeyboardInterrupt:
        node.publish_stop()
        node.get_logger().info("用户中断")
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
