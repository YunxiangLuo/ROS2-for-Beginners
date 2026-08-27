import numpy as np
import rclpy

from .math_utils import euler_from_quaternion, quaternion_from_euler, rotation_matrix_from_euler


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("py_coordinate_transformation")

    v1 = np.array([1.0, 1.0, 1.0])
    v2 = np.array([1.0, 0.0, 1.0])
    node.get_logger().info(f"dot(v1, v2) = {float(np.dot(v1, v2)):.6f}")
    node.get_logger().info(f"norm(v2) = {float(np.linalg.norm(v2)):.6f}")
    node.get_logger().info(f"normalized(v2) = {v2 / np.linalg.norm(v2)}")

    q = quaternion_from_euler(0.0, 0.0, 1.57)
    node.get_logger().info(f"quaternion_from_euler = {q}")
    node.get_logger().info(f"euler_from_quaternion = {euler_from_quaternion(*q)}")
    node.get_logger().info(f"rotation_matrix = {rotation_matrix_from_euler(0.0, 0.0, 1.57)}")

    node.destroy_node()
    rclpy.shutdown()
