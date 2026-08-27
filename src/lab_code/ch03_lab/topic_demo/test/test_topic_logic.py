import pytest
from geometry_msgs.msg import Point

from topic_demo.publisher import make_gps_point
from topic_demo.square_driver import make_twist
from topic_demo.subscriber import distance_from_origin


def test_gps_point_and_distance_logic():
    point = make_gps_point(2.0)
    assert point.x == pytest.approx(2.0)
    assert point.y == pytest.approx(5.0)
    assert distance_from_origin(point) == pytest.approx(29 ** 0.5)


def test_square_driver_command_shape():
    msg = make_twist(0.2, 1.57)
    assert isinstance(msg, type(Point())) is False
    assert msg.linear.x == pytest.approx(0.2)
    assert msg.angular.z == pytest.approx(1.57)
