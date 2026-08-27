#!/usr/bin/env python3
"""Finish one Cartographer trajectory and save its pbstream state."""

import argparse
from pathlib import Path
import sys

from cartographer_ros_msgs.srv import FinishTrajectory, WriteState
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


class CartographerStateSaver(Node):
    def __init__(self):
        super().__init__('cartographer_state_saver')
        self.finish_client = self.create_client(
            FinishTrajectory,
            '/finish_trajectory',
        )
        self.write_client = self.create_client(WriteState, '/write_state')

    def wait_for_services(self):
        if not self.finish_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/finish_trajectory is unavailable')
            return False
        if not self.write_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/write_state is unavailable')
            return False
        return True

    def finish_trajectory(self, trajectory_id):
        request = FinishTrajectory.Request()
        request.trajectory_id = trajectory_id
        future = self.finish_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done() or future.result() is None:
            self.get_logger().error('Failed to finish trajectory')
            return False
        status = getattr(future.result(), 'status', None)
        if status is not None and status.code != 0:
            self.get_logger().error(f'Finish failed: {status.message}')
            return False
        return True

    def write_state(self, filename):
        request = WriteState.Request()
        request.filename = str(filename)
        request.include_unfinished_submaps = True
        future = self.write_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done() or future.result() is None:
            self.get_logger().error('Failed to write Cartographer state')
            return False
        status = getattr(future.result(), 'status', None)
        if status is not None and status.code != 0:
            self.get_logger().error(f'Write failed: {status.message}')
            return False
        return True


def parse_arguments(raw_args):
    parser = argparse.ArgumentParser()
    parser.add_argument('trajectory_id', type=int)
    parser.add_argument('output_file')
    return parser.parse_args(remove_ros_args(raw_args)[1:])


def main():
    raw_args = sys.argv
    parsed = parse_arguments(raw_args)
    output_file = Path(parsed.output_file).expanduser().resolve()
    output_file.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=raw_args)
    node = CartographerStateSaver()
    try:
        if not node.wait_for_services():
            return
        if not node.finish_trajectory(parsed.trajectory_id):
            return
        if node.write_state(output_file):
            node.get_logger().info(f'Saved Cartographer state: {output_file}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
