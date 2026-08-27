import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
import numpy as np

try:
    from av_carla_interfaces.msg import PerceptionObjectArray, PerceptionObject
    _HAS_PERCEPTION_MSG = True
except ImportError:
    _HAS_PERCEPTION_MSG = False
    PerceptionObjectArray = object
    PerceptionObject = object

try:
    from av_carla_interfaces.msg import ClusterArray
    _HAS_CLUSTER_MSG = True
except ImportError:
    _HAS_CLUSTER_MSG = False
    ClusterArray = object


class FusionNode(Node):

    def __init__(self):
        super().__init__('fusion_node')

        self.last_detections = []
        self.last_clusters = []

        self.det_sub = self.create_subscription(
            Detection2DArray, '/detections', self.detection_callback, 10)

        if _HAS_CLUSTER_MSG:
            self.cluster_sub = self.create_subscription(
                ClusterArray, '/clusters', self.cluster_callback, 10)
        else:
            self.cluster_sub = None
            self.get_logger().warn('av_carla_interfaces not found, /clusters subscription disabled')

        self.obj_pub = self.create_publisher(
            PerceptionObjectArray, '/perception_objects', 10)

        self.timer = self.create_timer(0.1, self.fusion_timer)

        self.K = np.array([[600.0, 0.0, 640.0],
                           [0.0, 600.0, 360.0],
                           [0.0, 0.0, 1.0]])
        self.image_width = 1280
        self.image_height = 720

        self.get_logger().info('FusionNode started')

    def detection_callback(self, msg):
        self.last_detections = msg.detections
        self.last_header = msg.header

    def cluster_callback(self, msg):
        self.last_clusters = msg.clusters

    def project_to_image(self, point_3d):
        pt = np.array([point_3d.x, point_3d.y, point_3d.z])
        # 相机后方的点(z<=0)不可见, 不投影(否则会镜像到图像内造成误匹配)
        if pt[2] <= 0.0:
            return None
        uv = self.K @ pt
        if uv[2] == 0:
            return None
        u = int(uv[0] / uv[2])
        v = int(uv[1] / uv[2])
        if 0 <= u < self.image_width and 0 <= v < self.image_height:
            return (u, v)
        return None

    def fusion_timer(self):
        if not hasattr(self, 'last_header'):
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        if not _HAS_PERCEPTION_MSG:
            return

        obj_array = PerceptionObjectArray()
        obj_array.header = header

        for cluster in self.last_clusters:
            centroid = cluster.centroid.position
            uv = self.project_to_image(centroid)
            if uv is None:
                continue

            matched = False
            for det in self.last_detections:
                dcx = det.bbox.center.position.x
                dcy = det.bbox.center.position.y
                du, dv = uv
                dist = np.hypot(dcx - du, dcy - dv)
                if dist < 100.0:
                    obj = PerceptionObject()
                    obj.pose = cluster.centroid
                    obj.id = det.results[0].hypothesis.class_id if det.results else 'unknown'
                    obj.confidence = det.results[0].hypothesis.score if det.results else 0.0
                    obj_array.objects.append(obj)
                    matched = True
                    break

            if not matched:
                obj = PerceptionObject()
                obj.pose = cluster.centroid
                obj.id = 'obstacle'
                obj.confidence = 0.5
                obj_array.objects.append(obj)

        self.obj_pub.publish(obj_array)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


