import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Image, PointCloud2, NavSatFix, Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger

from dataclasses import dataclass, field
from typing import Dict, Any, Optional


@dataclass
class SensorStatus:
    name: str
    type: str
    active: bool = False
    last_msg_time: Optional[float] = None
    extra: Dict[str, Any] = field(default_factory=dict)


class SensorManager(Node):

    def __init__(self):
        super().__init__('av_sensor_manager')
        self.cbg = ReentrantCallbackGroup()

        self.sensor_statuses: Dict[str, SensorStatus] = {}
        self._subs = []

        self.srv = self.create_service(
            Trigger, '~/reconfigure_sensors', self.reconfigure_callback,
            callback_group=self.cbg,
        )

        self.status_pub = self.create_publisher(
            String, '~/sensor_status', 10,
        )

        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('SensorManager started')

    def publish_status(self):
        """周期性发布所有传感器的健康状态摘要。"""
        msg = String()
        lines = []
        for name, status in self.sensor_statuses.items():
            age = None
            if status.last_msg_time is not None:
                age = max(0.0, self.get_clock().now().nanoseconds / 1e9 - status.last_msg_time)
            lines.append(
                f'{name}[{status.type}]: '
                f'{"active" if status.active else "inactive"}'
                + (f' age={age:.2f}s' if age is not None else '')
            )
        msg.data = '; '.join(lines) if lines else 'no sensors configured'
        self.status_pub.publish(msg)

    def reconfigure_callback(self, request, response):
        self.get_logger().info('Reconfigure triggered')
        response.success = True
        response.message = 'Reconfigure not yet implemented'
        return response

    def _subscribe_sensor(self, topic: str, msg_type, sensor_name: str):
        sub = self.create_subscription(
            msg_type, topic,
            lambda msg, name=sensor_name: self._sensor_callback(msg, name),
            10, callback_group=self.cbg,
        )
        self._subs.append(sub)

    def _sensor_callback(self, msg, name: str):
        if name in self.sensor_statuses:
            self.sensor_statuses[name].active = True
            self.sensor_statuses[name].last_msg_time = self.get_clock().now().nanoseconds / 1e9

    def setup_default_sensors(self):
        topics = {
            '/carla/ego_vehicle/rgb/front': (Image, 'front_rgb'),
            '/carla/ego_vehicle/lidar': (PointCloud2, 'lidar'),
            '/carla/ego_vehicle/gnss': (NavSatFix, 'gnss'),
            '/carla/ego_vehicle/imu': (Imu, 'imu'),
        }
        for topic, (msg_type, name) in topics.items():
            self.sensor_statuses[name] = SensorStatus(name=name, type=msg_type.__name__)
            self._subscribe_sensor(topic, msg_type, name)
        self.get_logger().info(f'Subscribed to {len(topics)} sensor topics')


def main(args=None):
    rclpy.init(args=args)
    node = SensorManager()
    node.setup_default_sensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

