from unittest.mock import Mock

import pytest
import rclpy

from sensor_pub.sensor_publisher import SensorPublisher


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_sensor_publisher_callback_populates_interface_message():
    node = SensorPublisher()
    node.pub = Mock()
    try:
        node.callback()
        message = node.pub.publish.call_args.args[0]
        assert message.temperature == pytest.approx(25.5)
        assert message.humidity == pytest.approx(60.0)
        assert message.pressure == pytest.approx(1013.25)
        assert message.device_id == 'sensor_01'
    finally:
        node.destroy_node()
