# -*- coding: utf-8 -*-
"""av_perception_py 单元测试：FusionNode 相机投影。"""

import numpy as np
import pytest

from av_perception_py.fusion_node import FusionNode


def make_fusion():
    node = FusionNode.__new__(FusionNode)
    node.K = np.array([[600.0, 0.0, 640.0],
                       [0.0, 600.0, 360.0],
                       [0.0, 0.0, 1.0]])
    node.image_width = 1280
    node.image_height = 720
    return node


class _P:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def test_project_center():
    node = make_fusion()
    # z 轴上 2m 处的点 -> 光轴中心 (640, 360)
    assert node.project_to_image(_P(0.0, 0.0, 2.0)) == (640, 360)


def test_project_off_center():
    node = make_fusion()
    uv = node.project_to_image(_P(1.0, -1.0, 2.0))
    # x=1,z=2 -> u=640+600/2=940; y=-1,z=2 -> v=360-600/2=60
    assert uv == (940, 60)


def test_project_outside_image_returns_none():
    node = make_fusion()
    assert node.project_to_image(_P(100.0, 0.0, 1.0)) is None
    assert node.project_to_image(_P(0.0, 100.0, 1.0)) is None


def test_project_behind_camera_returns_none():
    node = make_fusion()
    assert node.project_to_image(_P(0.0, 0.0, -2.0)) is None
    assert node.project_to_image(_P(0.0, 0.0, 0.0)) is None
