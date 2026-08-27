import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header, ColorRGBA
import numpy as np
import struct

try:
    from av_carla_interfaces.msg import ClusterArray, Cluster
    _HAS_CLUSTER_MSG = True
except ImportError:
    _HAS_CLUSTER_MSG = False
    ClusterArray = object
    Cluster = object


_POINTFIELD_DTYPES = {}


def _register_pointfield_dtypes():
    """建立 PointField 数据类型枚举到 numpy dtype 的映射(真实 ROS 或 stub 均可)。"""
    mapping = [
        ('FLOAT32', np.float32), ('FLOAT64', np.float64),
        ('INT32', np.int32), ('UINT32', np.uint32),
        ('INT16', np.int16), ('UINT16', np.uint16),
        ('INT8', np.int8), ('UINT8', np.uint8),
    ]
    for name, dt in mapping:
        enum_val = getattr(PointField, name, None)
        if enum_val is not None:
            _POINTFIELD_DTYPES[enum_val] = dt


_register_pointfield_dtypes()


def pointcloud2_to_numpy(msg):
    """将 PointCloud2 消息解析为 numpy structured array。

    使用各字段的 offset 构造 dtype，正确处理字段间有 padding 的点云。
    """
    if not msg.fields:
        return np.array([], dtype=np.float32)

    names, formats, offsets = [], [], []
    for field in msg.fields:
        dt = _POINTFIELD_DTYPES.get(field.datatype, np.float32)
        names.append(field.name)
        formats.append(dt)
        offsets.append(field.offset)

    min_itemsize = max(o + np.dtype(f).itemsize for o, f in zip(offsets, formats))
    # 真实 PointCloud2 携带 point_step(含行尾 padding), 优先使用
    point_step = int(getattr(msg, 'point_step', 0) or 0)
    itemsize = max(point_step, min_itemsize)
    dtype = np.dtype({
        'names': names, 'formats': formats,
        'offsets': offsets, 'itemsize': itemsize,
    })
    return np.frombuffer(bytes(msg.data), dtype=dtype)


class LidarDetector(Node):

    def __init__(self):
        super().__init__('lidar_detector')

        self.declare_parameter('voxel_size', 0.2)
        self.declare_parameter('cluster_tolerance', 0.5)
        self.declare_parameter('min_cluster_size', 10)
        self.declare_parameter('max_cluster_size', 1000)

        self.voxel_size = self.get_parameter('voxel_size').value
        self.cluster_tolerance = self.get_parameter('cluster_tolerance').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.max_cluster_size = self.get_parameter('max_cluster_size').value

        self.sub = self.create_subscription(
            PointCloud2, '/carla/ego_vehicle/lidar_top/pointcloud',
            self.pointcloud_callback, 10)

        self.marker_pub = self.create_publisher(MarkerArray, '/lidar_obstacle_markers', 10)
        self.pose_pub = self.create_publisher(PoseArray, '/lidar_obstacle_poses', 10)

        if _HAS_CLUSTER_MSG:
            self.cluster_pub = self.create_publisher(ClusterArray, '/clusters', 10)
        else:
            self.cluster_pub = None
            self.get_logger().warn('av_carla_interfaces not found, /clusters publisher disabled')

        self.get_logger().info('LidarDetector node started')

    def pointcloud_callback(self, msg):
        try:
            pc = pointcloud2_to_numpy(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to parse pointcloud: {e}')
            return

        names = pc.dtype.names

        x = pc['x'] if 'x' in names else np.zeros(len(pc))
        y = pc['y'] if 'y' in names else np.zeros(len(pc))
        z = pc['z'] if 'z' in names else np.zeros(len(pc))

        mask = (x > -50) & (x < 50) & (y > -50) & (y < 50) & (z > -2) & (z < 5)
        x, y, z = x[mask], y[mask], z[mask]

        if len(x) == 0:
            return

        points = np.vstack([x, y, z]).T

        points = self._voxel_filter(points, self.voxel_size)
        clusters = self._dbscan_clustering(points, self.cluster_tolerance, self.min_cluster_size)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id

        markers = MarkerArray()
        pose_array = PoseArray()
        pose_array.header = header

        if _HAS_CLUSTER_MSG and self.cluster_pub is not None:
            cluster_array = ClusterArray()
            cluster_array.header = header

        for i, cluster_points in enumerate(clusters):
            if len(cluster_points) < self.min_cluster_size:
                continue
            if len(cluster_points) > self.max_cluster_size:
                continue

            centroid = cluster_points.mean(axis=0)

            pose = Pose()
            pose.position = Point(x=float(centroid[0]), y=float(centroid[1]), z=float(centroid[2]))
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            pose_array.poses.append(pose)

            marker = Marker()
            marker.header = header
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose

            extent = cluster_points.max(axis=0) - cluster_points.min(axis=0)
            marker.scale.x = max(float(extent[0]), 0.2)
            marker.scale.y = max(float(extent[1]), 0.2)
            marker.scale.z = max(float(extent[2]), 0.2)

            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.4)
            marker.lifetime.sec = 1
            markers.markers.append(marker)

            if _HAS_CLUSTER_MSG and self.cluster_pub is not None:
                cluster = Cluster()
                cluster.centroid = pose
                for pt in cluster_points:
                    point_msg = Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2]))
                    cluster.points.append(point_msg)
                cluster_array.clusters.append(cluster)

        self.marker_pub.publish(markers)
        self.pose_pub.publish(pose_array)
        if _HAS_CLUSTER_MSG and self.cluster_pub is not None:
            self.cluster_pub.publish(cluster_array)

    def _voxel_filter(self, points, voxel_size):
        if len(points) == 0:
            return points
        indices = np.floor(points / voxel_size).astype(int)
        _, unique_idx = np.unique(indices, axis=0, return_index=True)
        return points[unique_idx]

    def _dbscan_clustering(self, points, eps, min_samples):
        """标准 DBSCAN 聚类(标签式实现)。

        - 核心点邻域不足时保持噪声状态, 之后仍可被邻近簇吸收为边界点;
        - 返回点数 >= min_samples 的簇(点数组列表)。
        """
        n = len(points)
        if n == 0:
            return []

        labels = [-1] * n  # -1 表示噪声/未访问, 否则为簇编号
        cluster_id = 0

        def region_query(idx):
            dists = np.linalg.norm(points - points[idx], axis=1)
            return np.where(dists <= eps)[0].tolist()

        for i in range(n):
            if labels[i] != -1:
                continue
            neighbors = region_query(i)
            if len(neighbors) < min_samples:
                continue  # 暂记为噪声, 后续可能被吸收

            labels[i] = cluster_id
            queue = [j for j in neighbors if j != i]
            while queue:
                j = queue.pop()
                if labels[j] != -1:
                    continue
                labels[j] = cluster_id
                j_neighbors = region_query(j)
                if len(j_neighbors) >= min_samples:
                    queue.extend(k for k in j_neighbors if labels[k] == -1)
            cluster_id += 1

        clusters = []
        for cid in range(cluster_id):
            members = np.array([i for i in range(n) if labels[i] == cid], dtype=int)
            if len(members) >= min_samples:
                clusters.append(points[members])
        return clusters


def main(args=None):
    rclpy.init(args=args)
    node = LidarDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



