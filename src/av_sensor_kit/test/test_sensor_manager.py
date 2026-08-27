# -*- coding: utf-8 -*-
"""av_sensor_kit 单元测试：SensorManager 状态跟踪逻辑(不依赖真实 ROS)。"""

from av_sensor_kit.sensor_manager import SensorManager, SensorStatus


def make_manager():
    node = SensorManager.__new__(SensorManager)
    node.sensor_statuses = {}
    node.subscriptions = []
    return node


def test_sensor_status_defaults():
    s = SensorStatus(name='front_rgb', type='Image')
    assert s.active is False
    assert s.last_msg_time is None
    assert s.extra == {}


def test_sensor_callback_marks_active():
    node = make_manager()
    node.sensor_statuses['lidar'] = SensorStatus(name='lidar', type='PointCloud2')

    class _FakeClock:
        def now(self):
            class _T:
                nanoseconds = 1_700_000_000_000_000_000
            return _T()

    node.get_clock = lambda: _FakeClock()
    node._sensor_callback(None, 'lidar')
    assert node.sensor_statuses['lidar'].active is True
    assert node.sensor_statuses['lidar'].last_msg_time == 1.7e9


def test_sensor_callback_ignores_unknown_sensor():
    node = make_manager()
    node._sensor_callback(None, 'unknown_sensor')
    assert 'unknown_sensor' not in node.sensor_statuses


def test_setup_default_sensors_registers_four():
    node = make_manager()
    created = []

    def fake_subscribe(topic, msg_type, name):
        created.append((topic, msg_type, name))
        node.sensor_statuses[name] = SensorStatus(name=name, type=msg_type.__name__)

    node._subscribe_sensor = fake_subscribe
    node.get_logger = lambda: type('L', (), {'info': lambda *a, **k: None})()

    node.setup_default_sensors()
    assert len(created) == 4
    topics = [t for t, _, _ in created]
    assert '/carla/ego_vehicle/rgb/front' in topics
    assert '/carla/ego_vehicle/lidar' in topics
    assert set(node.sensor_statuses) == {'front_rgb', 'lidar', 'gnss', 'imu'}
