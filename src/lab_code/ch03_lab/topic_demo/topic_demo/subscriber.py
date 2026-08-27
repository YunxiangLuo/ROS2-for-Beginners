import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


def distance_from_origin(point):
    """Return the planar distance represented by a GPS point."""
    return (point.x ** 2 + point.y ** 2) ** 0.5


class GpsSubscriber(Node):
    """GPS数据订阅节点 — 监听/gps_position话题并处理坐标"""

    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(
            Point,
            '/gps_position',
            self.position_callback,
            10)
        self.subscription  # 防止 lint 警告

    def position_callback(self, msg):
        distance = distance_from_origin(msg)
        self.get_logger().info(
            f'收到位置: ({msg.x:.2f}, {msg.y:.2f}), '
            f'距原点: {distance:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = GpsSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
