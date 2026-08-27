"""Tests for the param_demo parameter validation logic."""
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

from param_demo.param_demo import ParamDemoNode


def test_validate_rejects_negative_max_speed():
    rclpy.init()
    try:
        node = ParamDemoNode()
        result = node.validate_params([Parameter('max_speed', Parameter.Type.DOUBLE, -1.0)])
        assert isinstance(result, SetParametersResult)
        assert not result.successful
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_validate_rejects_overspeed_max_speed():
    rclpy.init()
    try:
        node = ParamDemoNode()
        result = node.validate_params([Parameter('max_speed', Parameter.Type.DOUBLE, 99.0)])
        assert not result.successful
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_validate_accepts_valid_max_speed():
    rclpy.init()
    try:
        node = ParamDemoNode()
        result = node.validate_params([Parameter('max_speed', Parameter.Type.DOUBLE, 3.5)])
        assert result.successful
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_validate_rejects_unknown_mode():
    rclpy.init()
    try:
        node = ParamDemoNode()
        result = node.validate_params([Parameter('mode', Parameter.Type.STRING, 'teleop')])
        assert not result.successful
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_validate_accepts_known_mode():
    rclpy.init()
    try:
        node = ParamDemoNode()
        for value in ('auto', 'manual', 'hybrid'):
            result = node.validate_params([Parameter('mode', Parameter.Type.STRING, value)])
            assert result.successful, f"mode={value} should be accepted"
        node.destroy_node()
    finally:
        rclpy.shutdown()
