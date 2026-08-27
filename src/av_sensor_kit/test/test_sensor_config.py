# -*- coding: utf-8 -*-
"""av_sensor_kit 单元测试：传感器配置的序列化/反序列化与预设配置。"""

import os

import pytest
import yaml

from av_sensor_kit.sensor_config import (
    SensorConfig, load_config, save_config,
    FRONT_CAMERA, LIDAR_64, GNSS, IMU,
)

CONFIG_DIR = os.path.join(os.path.dirname(__file__), '..', 'config')


def test_to_dict_drops_none_fields():
    cfg = SensorConfig(type='sensor.other.gnss', name='gnss')
    d = cfg.to_dict()
    # 未设置的可选字段(width/height/fov/...)不应出现在字典中
    assert d == {'type': 'sensor.other.gnss', 'name': 'gnss',
                 'x': 0.0, 'y': 0.0, 'z': 0.0,
                 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}


def test_to_dict_keeps_optional_fields():
    d = FRONT_CAMERA.to_dict()
    assert d['width'] == 1280 and d['height'] == 720 and d['fov'] == 90
    assert d['type'] == 'sensor.camera.rgb'
    assert d['name'] == 'front_rgb'


def test_save_load_roundtrip(tmp_path):
    configs = [FRONT_CAMERA, LIDAR_64, GNSS, IMU]
    path = str(tmp_path / 'sensors.yaml')
    save_config(configs, path)
    loaded = load_config(path)
    assert len(loaded) == len(configs)
    for orig, new in zip(configs, loaded):
        assert orig.to_dict() == new.to_dict()


def test_load_default_config_file():
    """随包发布的 config/default_sensors.yaml 必须可被 load_config 解析。"""
    path = os.path.join(CONFIG_DIR, 'default_sensors.yaml')
    assert os.path.isfile(path), 'default_sensors.yaml missing'
    configs = load_config(path)
    assert len(configs) > 0
    for c in configs:
        assert c.type and c.name
        assert isinstance(c.x, float)


def test_presets_sanity():
    assert LIDAR_64.channels == 64 and LIDAR_64.range == 120.0
    assert GNSS.type == 'sensor.other.gnss'
    assert IMU.type == 'sensor.other.imu'
