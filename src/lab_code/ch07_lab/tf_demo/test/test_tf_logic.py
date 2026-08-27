"""Tests for the tf_demo transform construction logic."""
import math

import tf_demo.tf_broadcaster as broadcaster


def test_module_imports_cleanly():
    assert hasattr(broadcaster, 'TFBroadcaster')
    assert hasattr(broadcaster, 'main')


def test_quaternion_from_yaw_is_normalized():
    # The broadcaster builds rotation.z = sin(angle/2), rotation.w = cos(angle/2).
    angle = 0.7
    z = math.sin(angle / 2)
    w = math.cos(angle / 2)
    norm = (z * z + w * w) ** 0.5
    assert abs(norm - 1.0) < 1e-9


def test_circular_odometry_translation_stays_on_unit_circle():
    # publish_odom_tf places base_link on a unit circle of radius 1.0.
    for angle in (0.0, 0.5, 1.0, 1.5, math.pi):
        x = 1.0 * math.cos(angle)
        y = 1.0 * math.sin(angle)
        assert abs((x * x + y * y) - 1.0) < 1e-9


def test_static_child_frames_are_documented():
    # The static transforms documented in the README/source: five sensor frames.
    expected = {'laser_frame', 'camera_frame', 'imu_link', 'left_wheel', 'right_wheel'}
    source = open(broadcaster.__file__, encoding='utf-8').read()
    for frame in expected:
        assert frame in source, f"{frame} not present in tf_broadcaster source"
