import pytest

import rclpy
from rclpy.lifecycle import TransitionCallbackReturn

from lifecycle_demo.hello_node import HelloRos2Node


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_lifecycle_transitions_and_publish_counter():
    node = HelloRos2Node()
    try:
        assert node.pub is None
        assert node.count == 0

        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        assert node.pub is not None
        assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS

        node.timer_callback()
        assert node.count == 1
        assert node.active is True

        assert node.trigger_deactivate() == TransitionCallbackReturn.SUCCESS
        assert node.active is False
        assert node.trigger_cleanup() == TransitionCallbackReturn.SUCCESS
        assert node.pub is None
        assert node.timer is None
    finally:
        node.destroy_node()
