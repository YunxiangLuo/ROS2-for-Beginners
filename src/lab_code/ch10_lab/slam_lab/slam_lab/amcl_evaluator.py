#!/usr/bin/env python3
"""Compare AMCL and simulator ground-truth poses in one TF frame."""

import math

import rclpy
import tf2_geometry_msgs  # Registers geometry message conversions with tf2.
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_pose(pose):
    q = pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class AmclEvaluator(Node):
    def __init__(self):
        super().__init__('amcl_evaluator')
        self.declare_parameter('ground_truth_topic', '/ground_truth/odom')
        self.declare_parameter('target_frame', 'map')

        ground_truth_topic = self.get_parameter('ground_truth_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.amcl_message = None
        self.ground_truth_message = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10,
        )
        self.ground_truth_sub = self.create_subscription(
            Odometry,
            ground_truth_topic,
            self.ground_truth_callback,
            10,
        )
        self.timer = self.create_timer(1.0, self.evaluate)
        self.get_logger().info(
            f'Comparing /amcl_pose with {ground_truth_topic} in '
            f'{self.target_frame}'
        )

    def amcl_callback(self, msg):
        self.amcl_message = msg

    def ground_truth_callback(self, msg):
        self.ground_truth_message = msg

    def transform_pose(self, header, pose):
        stamped = PoseStamped()
        stamped.header = header
        stamped.pose = pose
        return self.tf_buffer.transform(
            stamped,
            self.target_frame,
            timeout=Duration(seconds=0.2),
        )

    def evaluate(self):
        if self.amcl_message is None or self.ground_truth_message is None:
            return

        try:
            amcl_pose = self.transform_pose(
                self.amcl_message.header,
                self.amcl_message.pose.pose,
            )
            ground_truth_pose = self.transform_pose(
                self.ground_truth_message.header,
                self.ground_truth_message.pose.pose,
            )
        except TransformException as error:
            self.get_logger().warn(f'Cannot transform poses: {error}')
            return

        dx = amcl_pose.pose.position.x - ground_truth_pose.pose.position.x
        dy = amcl_pose.pose.position.y - ground_truth_pose.pose.position.y
        position_error = math.hypot(dx, dy)

        yaw_error = yaw_from_pose(amcl_pose.pose) - yaw_from_pose(
            ground_truth_pose.pose
        )
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        self.get_logger().info(
            f'position_error={position_error:.3f} m, '
            f'yaw_error={math.degrees(abs(yaw_error)):.2f} deg'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AmclEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
