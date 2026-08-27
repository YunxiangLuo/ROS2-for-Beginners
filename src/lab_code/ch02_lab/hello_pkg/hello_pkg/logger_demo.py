import rclpy
from rclpy.node import Node


class LoggerDemoNode(Node):
    """展示日志系统分级输出、节流和一次性输出"""

    def __init__(self):
        super().__init__('logger_demo')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.timer = self.create_timer(1.0, self.log_all_levels)
        self.counter = 0

    def log_all_levels(self):
        self.counter += 1
        self.get_logger().debug(f'DEBUG 消息 #{self.counter}')
        self.get_logger().info(f'INFO 消息 #{self.counter}')
        self.get_logger().warn(f'WARN 消息 #{self.counter}')

        if self.counter == 3:
            self.get_logger().error('ERROR: 第3次出现异常!', once=True)

        if self.counter >= 5:
            self.get_logger().warn(
                '高频警告——已节流', throttle_duration_sec=2)


def main(args=None):
    rclpy.init(args=args)
    node = LoggerDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
