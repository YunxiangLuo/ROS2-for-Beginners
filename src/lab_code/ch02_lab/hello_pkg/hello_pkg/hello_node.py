import rclpy
from rclpy.node import Node


class HelloNode(Node):
    """周期输出问候语的节点 — 演示定时器和日志功能"""

    def __init__(self):
        super().__init__('hello_node')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
        self.get_logger().info('HelloNode 已启动！', once=True)

    def timer_callback(self):
        self.count += 1
        self.get_logger().info(f'Hello ROS 2! 计数: {self.count}',
                               throttle_duration_sec=1)


def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
