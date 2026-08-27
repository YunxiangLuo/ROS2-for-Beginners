import math

import pytest
import rclpy
from geometry_msgs.msg import Quaternion

from hello_pkg.hello_node import HelloNode
from hello_pkg.odom_monitor import OdomMonitor


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_hello_node_timer_increments_counter():
    node = HelloNode()
    try:
        node.timer_callback()
        assert node.count == 1
    finally:
        node.destroy_node()


def test_quaternion_to_yaw_uses_w_component():
    orientation = Quaternion()
    orientation.z = math.sin(math.pi / 4.0)
    orientation.w = math.cos(math.pi / 4.0)

    assert OdomMonitor.quaternion_to_yaw(orientation) == pytest.approx(
        math.pi / 2.0)
