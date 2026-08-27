import rclpy
from rclpy.node import Node
from sensor_interfaces.msg import SensorData


class SensorPublisher(Node):
    """使用自定义SensorData消息的发布节点"""

    def __init__(self):
        super().__init__('sensor_publisher')
        self.pub = self.create_publisher(
            SensorData, '/sensor_data', 10)
        self.timer = self.create_timer(1.0, self.callback)

    def callback(self):
        """构造并发布 SensorData 消息"""
        msg = SensorData()
        msg.temperature = 25.5
        msg.humidity = 60.0
        msg.pressure = 1013.25
        msg.device_id = 'sensor_01'
        self.pub.publish(msg)
        self.get_logger().info(f'发布传感器数据: {msg.device_id}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
