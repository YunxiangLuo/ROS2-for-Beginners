# -*- coding: utf-8 -*-
"""av_safety_monitor 单元测试：TTC 计算、报警分级、紧急制动、故障注入。"""

import math
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from av_safety_monitor.safety_monitor import SafetyMonitor, AlertLevel, LEVEL_NAMES
from av_safety_monitor.fault_injector import FaultInjector


def make_monitor(ttc_warn=4.0, ttc_crit=2.5, ttc_emerg=1.5):
    node = SafetyMonitor.__new__(SafetyMonitor)
    node.ttc_warn = ttc_warn
    node.ttc_crit = ttc_crit
    node.ttc_emerg = ttc_emerg
    node.lane_warn = 0.5
    node.lane_crit = 1.0
    node.brake_decel = -5.0
    node._current_alert = AlertLevel.WARNING
    node._ego_speed = 0.0
    node._plan = None
    node._nearest_obstacle_distance = float('inf')
    node._nearest_obstacle_velocity = 0.0
    node._collision_reported = False
    node._emergency_pub = MagicMock()
    node._status_pub = MagicMock()
    node._marker_pub = MagicMock()
    node.get_logger = lambda: SimpleNamespace(
        info=lambda *a, **k: None, warn=lambda *a, **k: None,
        warning=lambda *a, **k: None, error=lambda *a, **k: None,
        fatal=lambda *a, **k: None)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: None))
    return node


class TestTTC:

    def test_no_obstacle_is_infinite(self):
        node = make_monitor()
        assert node._compute_ttc() == float('inf')

    def test_closing_obstacle(self):
        node = make_monitor()
        node._ego_speed = 10.0
        node._nearest_obstacle_velocity = 4.0
        node._nearest_obstacle_distance = 30.0
        assert node._compute_ttc() == pytest.approx(5.0)

    def test_static_obstacle(self):
        node = make_monitor()
        node._ego_speed = 10.0
        node._nearest_obstacle_distance = 25.0
        assert node._compute_ttc() == pytest.approx(2.5)

    def test_receding_obstacle_is_infinite(self):
        node = make_monitor()
        node._ego_speed = 5.0
        node._nearest_obstacle_velocity = 8.0  # 障碍更快远离
        assert node._compute_ttc() == float('inf')

    def test_zero_distance_is_zero(self):
        node = make_monitor()
        node._ego_speed = 5.0
        node._nearest_obstacle_distance = 0.0
        assert node._compute_ttc() == 0.0


class TestMonitorLoop:

    def test_normal_state_no_emergency(self):
        node = make_monitor()
        node._monitor_loop()
        node._emergency_pub.publish.assert_not_called()
        node._status_pub.publish.assert_called_once()

    def test_emergency_ttc_triggers_stop(self):
        node = make_monitor()
        node._ego_speed = 10.0
        node._nearest_obstacle_distance = 10.0  # TTC=1.0s < 1.5
        node._monitor_loop()
        node._emergency_pub.publish.assert_called_once()
        status = node._status_pub.publish.call_args[0][0]
        assert 'EMERGENCY' in status.data

    def test_critical_ttc_no_stop(self):
        node = make_monitor()
        node._ego_speed = 10.0
        node._nearest_obstacle_distance = 20.0  # TTC=2.0s: CRITICAL(>=1.5)
        node._monitor_loop()
        node._emergency_pub.publish.assert_not_called()
        status = node._status_pub.publish.call_args[0][0]
        assert 'CRITICAL' in status.data

    def test_warning_ttc(self):
        node = make_monitor()
        node._ego_speed = 10.0
        node._nearest_obstacle_distance = 35.0  # TTC=3.5s: WARNING
        node._monitor_loop()
        status = node._status_pub.publish.call_args[0][0]
        assert 'WARNING' in status.data
        node._emergency_pub.publish.assert_not_called()

    def test_collision_forces_emergency(self):
        node = make_monitor()
        node._collision_reported = True
        node._monitor_loop()
        node._emergency_pub.publish.assert_called_once()
        status = node._status_pub.publish.call_args[0][0]
        assert 'Collision' in status.data
        # 碰撞标志被消费
        assert node._collision_reported is False


class TestPerceptionCallback:

    def test_nearest_obstacle_updated(self):
        node = make_monitor()
        msg = SimpleNamespace(objects=[
            SimpleNamespace(
                pose=SimpleNamespace(position=SimpleNamespace(x=30.0, y=40.0)),
                velocity=SimpleNamespace(x=0.0, y=0.0)),
            SimpleNamespace(
                pose=SimpleNamespace(position=SimpleNamespace(x=3.0, y=4.0)),
                velocity=SimpleNamespace(x=1.0, y=2.0)),
        ])
        node._perception_callback(msg)
        assert node._nearest_obstacle_distance == pytest.approx(5.0)
        assert node._nearest_obstacle_velocity == pytest.approx(math.sqrt(5.0))

    def test_empty_objects_resets(self):
        node = make_monitor()
        node._nearest_obstacle_distance = 10.0
        node._perception_callback(SimpleNamespace(objects=[]))
        assert node._nearest_obstacle_distance == float('inf')


class TestFaultInjector:

    def make_injector(self, fault_type='drop', prob=1.0):
        node = FaultInjector.__new__(FaultInjector)
        node._fault_type = fault_type
        node._target_topic = '/plan'
        node._fault_probability = prob
        node._active = True
        node._stall_active = False
        node._buffer = []
        node._pub = MagicMock()
        node.get_logger = lambda: SimpleNamespace(
            info=lambda *a, **k: None, warn=lambda *a, **k: None,
            warning=lambda *a, **k: None, error=lambda *a, **k: None)
        return node

    def test_drop_always(self):
        node = self.make_injector('drop', prob=1.0)
        msg = SimpleNamespace(data='hello')
        node._fault_sub_callback(msg)
        node._pub.publish.assert_not_called()

    def test_drop_never_forwards(self):
        node = self.make_injector('drop', prob=0.0)
        msg = SimpleNamespace(data='hello')
        node._fault_sub_callback(msg)
        node._pub.publish.assert_called_once_with(msg)

    def test_inactive_forwards_all(self):
        node = self.make_injector('drop', prob=1.0)
        node._active = False
        msg = SimpleNamespace(data='hello')
        node._fault_sub_callback(msg)
        node._pub.publish.assert_called_once_with(msg)

    def test_noise_appends_marker(self):
        node = self.make_injector('noise', prob=1.0)
        node._fault_sub_callback(SimpleNamespace(data='hello'))
        node._pub.publish.assert_called_once()
        sent = node._pub.publish.call_args[0][0]
        assert sent.data.startswith('hello')
        assert 'noise' in sent.data

    def test_bias_appends_marker(self):
        node = self.make_injector('bias', prob=1.0)
        node._fault_sub_callback(SimpleNamespace(data='x'))
        sent = node._pub.publish.call_args[0][0]
        assert 'bias' in sent.data
