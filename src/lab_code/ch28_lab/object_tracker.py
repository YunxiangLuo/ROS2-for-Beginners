#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA, Header
from nav_msgs.msg import Path
import numpy as np
from filterpy.kalman import KalmanFilter
from scipy.optimize import linear_sum_assignment


class TrackObject:
    def __init__(self, track_id, x, y, class_name='unknown', confidence=0.0):
        self.id = track_id
        self.class_name = class_name
        self.confidence = confidence

        self.kf = KalmanFilter(dim_x=6, dim_z=2)
        dt = 0.1
        self.kf.F = np.array([
            [1, 0, dt, 0, 0, 0],
            [0, 1, 0, dt, 0, 0],
            [0, 0, 1, 0, dt, 0],
            [0, 0, 0, 1, 0, dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ])
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
        ])
        self.kf.P = np.eye(6) * 100
        self.kf.Q = np.eye(6) * 0.05
        self.kf.R = np.eye(2) * 1.0
        self.kf.x = np.array([x, y, 0, 0, 0, 0], dtype=float)

        self.lost_count = 0
        self.hit_count = 1
        self.max_lost = 10
        self.min_hits = 3
        self.is_confirmed = False

        self.path = [(x, y)]

    def predict(self):
        self.kf.predict()
        self.lost_count += 1
        return self.kf.x[:2]

    def update(self, x, y):
        self.kf.update(np.array([x, y]))
        self.lost_count = 0
        self.hit_count += 1
        if self.hit_count >= self.min_hits:
            self.is_confirmed = True
        self.path.append((self.kf.x[0], self.kf.x[1]))
        if len(self.path) > 50:
            self.path.pop(0)

    def get_state(self):
        return self.kf.x[:2], self.kf.x[2:4]


class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')

        self.declare_parameter('iou_threshold', 0.3)
        self.declare_parameter('max_lost', 10)
        self.declare_parameter('min_hits', 3)
        self.declare_parameter('track_timeout', 2.0)

        self.iou_thresh = self.get_parameter('iou_threshold').value
        self.max_lost = self.get_parameter('max_lost').value
        self.min_hits = self.get_parameter('min_hits').value
        self.track_timeout = self.get_parameter('track_timeout').value

        self.tracks = {}
        self.next_id = 1

        self.det_sub = self.create_subscription(
            MarkerArray, '/perception/obstacles/markers',
            self.detection_callback, 10
        )

        self.track_marker_pub = self.create_publisher(
            MarkerArray, '/perception/tracks/markers', 10
        )
        self.path_pub = self.create_publisher(
            MarkerArray, '/perception/tracks/paths', 10
        )

        self.timer = self.create_timer(0.1, self.maintain_tracks)
        self.latest_detections = []

        self.get_logger().info('Object tracker initialized')

    def detection_callback(self, msg):
        detections = []
        for marker in msg.markers:
            if marker.ns == 'obstacles':
                x = marker.pose.position.x
                y = marker.pose.position.y
                detections.append((x, y))

        self.latest_detections = detections
        self.update_tracks(detections)

    def update_tracks(self, detections):
        if not self.tracks and not detections:
            return

        track_ids = list(self.tracks.keys())
        track_positions = [self.tracks[tid].predict() for tid in track_ids]

        if not track_positions or not detections:
            for did in detections:
                self.create_track(did[0], did[1])
            self.cleanup_lost_tracks()
            return

        cost_matrix = np.zeros((len(detections), len(track_ids)))
        for i, (dx, dy) in enumerate(detections):
            for j, (tx, ty) in enumerate(track_positions):
                cost_matrix[i, j] = np.sqrt((dx - tx)**2 + (dy - ty)**2)

        if cost_matrix.size > 0:
            det_indices, trk_indices = linear_sum_assignment(cost_matrix)

            matched_dets = set()
            matched_trks = set()

            for di, tj in zip(det_indices, trk_indices):
                if cost_matrix[di, tj] < self.track_timeout:
                    tid = track_ids[tj]
                    self.tracks[tid].update(detections[di][0], detections[di][1])
                    matched_dets.add(di)
                    matched_trks.add(tj)

            for i in range(len(detections)):
                if i not in matched_dets:
                    self.create_track(detections[i][0], detections[i][1])

        self.cleanup_lost_tracks()
        self.publish_tracks()

    def create_track(self, x, y, class_name='unknown', confidence=0.0):
        track = TrackObject(self.next_id, x, y, class_name, confidence)
        track.max_lost = self.max_lost
        track.min_hits = self.min_hits
        self.tracks[self.next_id] = track
        self.next_id += 1
        self.get_logger().info(f'New track created: ID={track.id} at ({x:.2f}, {y:.2f})')

    def cleanup_lost_tracks(self):
        lost_ids = []
        for tid, track in self.tracks.items():
            if track.lost_count > track.max_lost:
                lost_ids.append(tid)

        for tid in lost_ids:
            self.get_logger().info(f'Track lost: ID={tid}, path length={len(self.tracks[tid].path)}')
            del self.tracks[tid]

    def maintain_tracks(self):
        if self.latest_detections:
            return

        for tid in list(self.tracks.keys()):
            self.tracks[tid].predict()

        self.cleanup_lost_tracks()
        self.publish_tracks()

    def publish_tracks(self):
        marker_array = MarkerArray()
        path_array = MarkerArray()

        for tid, track in self.tracks.items():
            state, velocity = track.get_state()

            marker = Marker()
            marker.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
            marker.ns = 'tracks'
            marker.id = tid
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(state[0])
            marker.pose.position.y = float(state[1])
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.6
            marker.scale.y = 0.6
            marker.scale.z = 0.6

            if track.is_confirmed:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            else:
                marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)

            marker.lifetime.sec = 1
            marker_array.markers.append(marker)

            text = Marker()
            text.header = marker.header
            text.ns = 'track_labels'
            text.id = tid
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(state[0])
            text.pose.position.y = float(state[1])
            text.pose.position.z = 1.5
            text.pose.orientation.w = 1.0
            text.scale.z = 0.5
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = f'ID:{tid} ({track.hit_count})'
            text.lifetime.sec = 1
            marker_array.markers.append(text)

            if len(track.path) >= 2:
                path_marker = Marker()
                path_marker.header = marker.header
                path_marker.ns = 'track_paths'
                path_marker.id = tid
                path_marker.type = Marker.LINE_STRIP
                path_marker.action = Marker.ADD
                path_marker.scale.x = 0.08
                path_marker.color = ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.6)
                path_marker.lifetime.sec = 2

                for px, py in track.path:
                    pt = Point(x=float(px), y=float(py), z=0.2)
                    path_marker.points.append(pt)

                path_array.markers.append(path_marker)

            vel_marker = Marker()
            vel_marker.header = marker.header
            vel_marker.ns = 'track_velocities'
            vel_marker.id = tid
            vel_marker.type = Marker.ARROW
            vel_marker.action = Marker.ADD
            vel_marker.points.append(Point(x=float(state[0]), y=float(state[1]), z=0.5))
            vel_marker.points.append(Point(
                x=float(state[0] + velocity[0] * 2),
                y=float(state[1] + velocity[1] * 2),
                z=0.5
            ))
            vel_marker.scale.x = 0.1
            vel_marker.scale.y = 0.15
            vel_marker.scale.z = 0.0
            vel_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            vel_marker.lifetime.sec = 1
            marker_array.markers.append(vel_marker)

        self.track_marker_pub.publish(marker_array)
        if path_array.markers:
            self.path_pub.publish(path_array)

        self.get_logger().info(
            f'Active tracks: {len(self.tracks)} | '
            f'Confirmed: {sum(1 for t in self.tracks.values() if t.is_confirmed)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
