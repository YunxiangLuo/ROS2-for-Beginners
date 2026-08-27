import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class QosPublisher(Node):
    """测试不同QoS配置的发布节点 — RELIABLE / BEST_EFFORT 双话题"""

    def __init__(self):
        super().__init__('qos_publisher')
        # 可靠传输 QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10)
        self.reliable_pub = self.create_publisher(
            String, '/qos_reliable', reliable_qos)

        # 尽力传输 QoS
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10)
        self.best_effort_pub = self.create_publisher(
            String, '/qos_best_effort', best_effort_qos)

        self.timer = self.create_timer(1.0, self.publish)
        self.count = 0

    def publish(self):
        self.count += 1
        rel_msg = String()
        rel_msg.data = f'RELIABLE: {self.count}'
        self.reliable_pub.publish(rel_msg)

        be_msg = String()
        be_msg.data = f'BEST_EFFORT: {self.count}'
        self.best_effort_pub.publish(be_msg)

        self.get_logger().info(f'发布 #{self.count}')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(QosPublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
