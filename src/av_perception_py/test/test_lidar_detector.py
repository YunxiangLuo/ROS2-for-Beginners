# -*- coding: utf-8 -*-
"""av_perception_py 单元测试：点云解析、体素滤波、DBSCAN 聚类、相机投影。"""

import struct

import numpy as np
import pytest

from sensor_msgs.msg import PointField

from av_perception_py.lidar_detector import pointcloud2_to_numpy, LidarDetector


class FakeField:
    def __init__(self, name, offset, datatype):
        self.name = name
        self.offset = offset
        self.datatype = datatype


class FakePointCloud2:
    def __init__(self, fields, data, point_step=0):
        self.fields = fields
        self.data = data
        self.point_step = point_step


def make_lidar():
    """绕过 ROS Node 初始化, 构造仅含算法状态的实例。"""
    node = LidarDetector.__new__(LidarDetector)
    node.voxel_size = 0.2
    node.cluster_tolerance = 0.5
    node.min_cluster_size = 3
    node.max_cluster_size = 1000
    return node


class TestPointcloudParsing:

    def test_packed_fields(self):
        """紧凑排列的 x/y/z float32 点云。"""
        fields = [
            FakeField('x', 0, PointField.FLOAT32),
            FakeField('y', 4, PointField.FLOAT32),
            FakeField('z', 8, PointField.FLOAT32),
        ]
        data = struct.pack('<fff', 1.0, 2.0, 3.0) + struct.pack('<fff', 4.0, 5.0, 6.0)
        pc = pointcloud2_to_numpy(FakePointCloud2(fields, data))
        assert len(pc) == 2
        assert pc['x'][0] == pytest.approx(1.0)
        assert pc['y'][1] == pytest.approx(5.0)
        assert pc['z'][0] == pytest.approx(3.0)

    def test_fields_with_padding(self):
        """带 padding 的点云(x@0,y@8,z@16, 步长24)必须按 offset 正确解析。"""
        fields = [
            FakeField('x', 0, PointField.FLOAT32),
            FakeField('y', 8, PointField.FLOAT32),
            FakeField('z', 16, PointField.FLOAT32),
        ]
        point0 = struct.pack('<f', 1.0) + b'\x00' * 4 + struct.pack('<f', 2.0) + b'\x00' * 4 + struct.pack('<f', 3.0) + b'\x00' * 4
        point1 = struct.pack('<f', 7.0) + b'\x00' * 4 + struct.pack('<f', 8.0) + b'\x00' * 4 + struct.pack('<f', 9.0) + b'\x00' * 4
        pc = pointcloud2_to_numpy(FakePointCloud2(fields, point0 + point1, point_step=24))
        assert len(pc) == 2
        # 若忽略 offset, y/z 会错读为 padding 的 0.0
        assert pc['x'][1] == pytest.approx(7.0)
        assert pc['y'][1] == pytest.approx(8.0)
        assert pc['z'][1] == pytest.approx(9.0)

    def test_empty_fields(self):
        pc = pointcloud2_to_numpy(FakePointCloud2([], b''))
        assert len(pc) == 0

    def test_mixed_types(self):
        fields = [
            FakeField('x', 0, PointField.FLOAT32),
            FakeField('intensity', 4, PointField.UINT8),
        ]
        data = struct.pack('<fB', 1.5, 200) + struct.pack('<fB', -2.5, 10)
        pc = pointcloud2_to_numpy(FakePointCloud2(fields, data))
        assert pc['intensity'][0] == 200
        assert pc['x'][1] == pytest.approx(-2.5)


class TestVoxelFilter:

    def test_empty(self):
        node = make_lidar()
        out = node._voxel_filter(np.zeros((0, 3)), 0.2)
        assert len(out) == 0

    def test_downsamples_dense_points(self):
        node = make_lidar()
        pts = np.tile(np.array([[1.0, 1.0, 1.0]]), (50, 1))  # 同一体素内 50 个点
        out = node._voxel_filter(pts, 0.2)
        assert len(out) == 1

    def test_keeps_distinct_voxels(self):
        node = make_lidar()
        pts = np.array([[0.0, 0, 0], [1.0, 0, 0], [2.0, 0, 0]])
        out = node._voxel_filter(pts, 0.2)
        assert len(out) == 3


class TestDBSCAN:

    def test_empty(self):
        node = make_lidar()
        assert node._dbscan_clustering(np.zeros((0, 3)), 0.5, 3) == []

    def test_two_clusters(self):
        node = make_lidar()
        cluster_a = np.tile([0.0, 0.0, 0.0], (10, 1)) + np.random.RandomState(0).rand(10, 3) * 0.05
        cluster_b = np.tile([10.0, 0.0, 0.0], (10, 1)) + np.random.RandomState(1).rand(10, 3) * 0.05
        pts = np.vstack([cluster_a, cluster_b])
        clusters = node._dbscan_clustering(pts, eps=0.5, min_samples=3)
        assert len(clusters) == 2
        sizes = sorted(len(c) for c in clusters)
        assert sizes == [10, 10]

    def test_border_points_absorbed(self):
        """边界点(自身邻域不足)必须被邻近簇吸收(旧实现缺陷回归测试)。

        构造: 3 个核心点彼此相邻 + 1 个只邻近核心点的边界点。
        """
        node = make_lidar()
        pts = np.array([
            [0.0, 0.0, 0.0],
            [0.3, 0.0, 0.0],
            [0.6, 0.0, 0.0],
            [0.8, 0.0, 0.0],  # 边界点: 邻域仅 2 个(<3), 但紧邻核心点
        ])
        clusters = node._dbscan_clustering(pts, eps=0.35, min_samples=3)
        assert len(clusters) == 1
        assert len(clusters[0]) == 4, 'border point must be absorbed into the cluster'

    def test_all_noise(self):
        node = make_lidar()
        pts = np.array([[0.0, 0, 0], [5.0, 0, 0], [10.0, 0, 0]])
        clusters = node._dbscan_clustering(pts, eps=0.5, min_samples=3)
        assert clusters == []

    def test_min_samples_filter(self):
        """点数不足 min_samples 的簇不返回。"""
        node = make_lidar()
        pts = np.array([[0.0, 0, 0], [0.1, 0, 0]])  # 互相在邻域内但都不足 3 个
        clusters = node._dbscan_clustering(pts, eps=0.5, min_samples=3)
        assert clusters == []

