#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN
import struct


class LidarObstacleDetector(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector')

        self.declare_parameter('lidar_topic', '/carla/ego_vehicle/lidar/point_cloud')
        self.declare_parameter('voxel_size', 0.2)
        self.declare_parameter('cluster_eps', 0.5)
        self.declare_parameter('min_cluster_size', 10)
        self.declare_parameter('max_cluster_size', 10000)
        self.declare_parameter('range_x_min', 0.0)
        self.declare_parameter('range_x_max', 50.0)
        self.declare_parameter('range_y_min', -20.0)
        self.declare_parameter('range_y_max', 20.0)
        self.declare_parameter('range_z_min', -2.0)
        self.declare_parameter('range_z_max', 3.0)

        self.voxel_size = self.get_parameter('voxel_size').value
        self.cluster_eps = self.get_parameter('cluster_eps').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.max_cluster_size = self.get_parameter('max_cluster_size').value

        self.x_min = self.get_parameter('range_x_min').value
        self.x_max = self.get_parameter('range_x_max').value
        self.y_min = self.get_parameter('range_y_min').value
        self.y_max = self.get_parameter('range_y_max').value
        self.z_min = self.get_parameter('range_z_min').value
        self.z_max = self.get_parameter('range_z_max').value

        self.sub = self.create_subscription(
            PointCloud2,
            self.get_parameter('lidar_topic').value,
            self.lidar_callback,
            10
        )

        self.obstacle_pub = self.create_publisher(
            PointCloud2, '/perception/obstacles/clusters', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/perception/obstacles/markers', 10
        )

        self.get_logger().info('LiDAR obstacle detector initialized')
        self.get_logger().info(
            f'Range: x[{self.x_min},{self.x_max}] y[{self.y_min},{self.y_max}] '
            f'z[{self.z_min},{self.z_max}]'
        )

    def lidar_callback(self, msg):
        points = self.extract_points(msg)
        if len(points) == 0:
            return

        points = self.pass_through_filter(points)
        if len(points) == 0:
            return

        points = self.voxel_filter(points)

        clusters, labels = self.dbscan_cluster(points)
        if len(clusters) == 0:
            return

        self.publish_obstacles(points, labels, msg.header)
        self.publish_markers(clusters, msg.header)

        self.get_logger().info(
            f'Points: {len(points)} → {len(clusters)} clusters'
        )

    def extract_points(self, msg):
        points = []
        try:
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                points.append([p[0], p[1], p[2]])
        except struct.error:
            return np.array([])
        return np.array(points)

    def pass_through_filter(self, points):
        if len(points) == 0:
            return points
        mask = (points[:, 0] >= self.x_min) & (points[:, 0] <= self.x_max) & \
               (points[:, 1] >= self.y_min) & (points[:, 1] <= self.y_max) & \
               (points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)
        return points[mask]

    def voxel_filter(self, points):
        if len(points) == 0:
            return points
        voxel_indices = np.floor(points[:, :3] / self.voxel_size).astype(np.int64)
        voxel_keys = voxel_indices[:, 0] * 1000000 + \
                     voxel_indices[:, 1] * 1000 + \
                     voxel_indices[:, 2]
        _, unique_idx = np.unique(voxel_keys, return_index=True)
        return points[unique_idx]

    def dbscan_cluster(self, points):
        if len(points) < self.min_cluster_size:
            return [], np.array([])

        clustering = DBSCAN(
            eps=self.cluster_eps,
            min_samples=int(self.min_cluster_size / 2),
            metric='euclidean',
            n_jobs=1
        ).fit(points[:, :2])

        labels = clustering.labels_
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)

        if n_clusters <= 0:
            return [], labels

        clusters = []
        for i in range(n_clusters):
            mask = labels == i
            cluster_size = np.sum(mask)
            if self.min_cluster_size <= cluster_size <= self.max_cluster_size:
                clusters.append(points[mask])

        return clusters, labels

    def publish_obstacles(self, points, labels, header):
        if len(points) == 0:
            return

        colored_points = np.ones((len(points), 4), dtype=np.float32)
        colored_points[:, :3] = points[:, :3]

        max_label = np.max(labels) if len(labels) > 0 and np.max(labels) >= 0 else 0
        for i in range(len(points)):
            if labels[i] >= 0:
                colored_points[i, 3] = float(labels[i]) / max(float(max_label), 1.0)
            else:
                colored_points[i, 3] = 0.0

        msg = pc2.create_cloud(
            header=header,
            fields=[
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ],
            points=colored_points.tolist()
        )
        self.obstacle_pub.publish(msg)

    def publish_markers(self, clusters, header):
        marker_array = MarkerArray()
        colors = [
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
            (1.0, 1.0, 0.0),
            (1.0, 0.0, 1.0),
            (0.0, 1.0, 1.0),
        ]

        for i, cluster in enumerate(clusters):
            centroid = np.mean(cluster, axis=0)
            min_bounds = np.min(cluster, axis=0)
            max_bounds = np.max(cluster, axis=0)
            size = max_bounds - min_bounds

            marker = Marker()
            marker.header = header
            marker.ns = 'obstacles'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(centroid[0])
            marker.pose.position.y = float(centroid[1])
            marker.pose.position.z = float(centroid[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = float(max(size[0], 0.3))
            marker.scale.y = float(max(size[1], 0.3))
            marker.scale.z = float(max(size[2], 0.3))

            c = colors[i % len(colors)]
            marker.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=0.5)
            marker.lifetime.sec = 1

            marker_array.markers.append(marker)

            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = 'obstacle_labels'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(centroid[0])
            text_marker.pose.position.y = float(centroid[1])
            text_marker.pose.position.z = float(max_bounds[2] + 0.5)
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.5
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f'C{i} ({len(cluster)} pts)'
            text_marker.lifetime.sec = 1

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
