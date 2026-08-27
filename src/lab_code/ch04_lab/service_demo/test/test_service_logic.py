import pytest
import rclpy
from example_interfaces.srv import AddTwoInts

from service_demo.server import AddTwoIntsServer, add_two_ints


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_add_two_ints_handles_positive_and_negative_values():
    assert add_two_ints(5, 3) == 8
    assert add_two_ints(-2, 7) == 5


def test_server_callback_populates_response():
    node = AddTwoIntsServer()
    try:
        request = AddTwoInts.Request()
        response = AddTwoInts.Response()
        request.a = 12
        request.b = -4

        result = node.handle_add(request, response)

        assert result.sum == 8
    finally:
        node.destroy_node()
