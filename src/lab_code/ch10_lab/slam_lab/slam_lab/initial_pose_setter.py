#!/usr/bin/env python3
"""Publish an AMCL initial pose from x, y and yaw command-line values."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


def make_message(node: Node, x: float, y: float, yaw: float):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = 'map'
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    message.pose.covariance[0] = 0.25
    message.pose.covariance[7] = 0.25
    message.pose.covariance[35] = 0.0685
    return message


def parse_arguments(raw_args):
    parser = argparse.ArgumentParser()
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('yaw_deg', type=float)
    return parser.parse_args(remove_ros_args(raw_args)[1:])


def main():
    raw_args = sys.argv
    parsed = parse_arguments(raw_args)
    rclpy.init(args=raw_args)

    node = Node('initial_pose_setter')
    publisher = node.create_publisher(
        PoseWithCovarianceStamped,
        '/initialpose',
        10,
    )

    deadline = time.monotonic() + 3.0
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if publisher.get_subscription_count() == 0:
        node.get_logger().warn(
            'No /initialpose subscriber found; check whether AMCL is active'
        )

    yaw = math.radians(parsed.yaw_deg)
    for _ in range(3):
        publisher.publish(make_message(node, parsed.x, parsed.y, yaw))
        rclpy.spin_once(node, timeout_sec=0.2)

    node.get_logger().info(
        f'Published initial pose: x={parsed.x:.2f}, y={parsed.y:.2f}, '
        f'yaw={parsed.yaw_deg:.1f} deg'
    )
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
