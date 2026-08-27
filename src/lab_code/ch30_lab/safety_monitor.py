#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from std_srvs.srv import Trigger
import numpy as np
import math
from enum import Enum, auto

INF = float('inf')


class CollisionLevel(Enum):
    SAFE = auto()
    WARNING = auto()
    CRITICAL = auto()


class DegradationLevel(Enum):
    NORMAL = auto()
    DEGRADED = auto()
    LIMITED = auto()
    SAFE_STOP = auto()


class SafetyMonitor(Node):

    def __init__(self):
        super().__init__('safety_monitor')

        self.declare_parameter('ttc_warning', 2.6)
        self.declare_parameter('ttc_brake', 1.8)
        self.declare_parameter('ttc_full_brake', 0.8)
        self.declare_parameter('deviation_threshold', 0.4)
        self.declare_parameter('lane_width', 3.5)
        self.declare_parameter('check_rate', 50.0)
        self.declare_parameter('ego_width', 2.0)
        self.declare_parameter('ego_length', 4.5)

        self.ttc_warning = self.get_parameter('ttc_warning').value
        self.ttc_brake = self.get_parameter('ttc_brake').value
        self.ttc_full_brake = self.get_parameter('ttc_full_brake').value
        self.deviation_threshold = self.get_parameter('deviation_threshold').value
        self.lane_width = self.get_parameter('lane_width').value
        self.ego_width = self.get_parameter('ego_width').value
        self.ego_length = self.get_parameter('ego_length').value

        self.ego_pose = None
        self.ego_velocity = 0.0
        self.objects = []
        self.planned_trajectory = None
        self.lane_center_pose = None

        self.collision_warning_pub = self.create_publisher(
            Bool, '/safety/collision_warning', 10)
        self.deviation_pub = self.create_publisher(
            Float32, '/safety/deviation', 10)
        self.aeb_cmd_pub = self.create_publisher(
            Twist, '/safety/aeb_command', 10)
        self.collision_level_pub = self.create_publisher(
            String, '/safety/collision_level', 10)
        self.visualization_pub = self.create_publisher(
            MarkerArray, '/safety/visualization', 10)

        self.objects_sub = self.create_subscription(
            Float32, '/perception/objects', self.objects_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/vehicle/odometry', self.odom_callback, 10)
        self.trajectory_sub = self.create_subscription(
            Path, '/planning/trajectory', self.trajectory_callback, 10)

        self.srv_group = ReentrantCallbackGroup()
        self.emergency_brake_srv = self.create_service(
            Trigger, '/safety/emergency_brake',
            self.emergency_brake_callback, callback_group=self.srv_group)

        check_period = 1.0 / self.get_parameter('check_rate').value
        self.timer = self.create_timer(check_period, self.monitor_loop)

        self.in_emergency = False
        self.brake_active = False
        self.cumulative_deviation = 0.0
        self.last_deviation_time = self.get_clock().now()

        self.get_logger().info('SafetyMonitor started')

    def objects_callback(self, msg):
        pass

    def odom_callback(self, msg):
        self.ego_pose = msg.pose.pose
        self.ego_velocity = math.sqrt(
            msg.twist.twist.linear.x ** 2 +
            msg.twist.twist.linear.y ** 2
        )

    def trajectory_callback(self, msg):
        self.planned_trajectory = msg

    def emergency_brake_callback(self, request, response):
        self.get_logger().warn('紧急制动服务触发!')
        self.in_emergency = True
        self.execute_emergency_brake()
        response.success = True
        response.message = 'Emergency brake executed'
        return response

    def monitor_loop(self):
        if self.ego_pose is None:
            return

        self.check_collisions()
        self.check_deviation()

    def compute_ttc(self, ego_pos, ego_vel, obj_pos, obj_vel):
        rel_pos = np.array([
            obj_pos.position.x - ego_pos.position.x,
            obj_pos.position.y - ego_pos.position.y
        ])
        rel_vel = np.array([
            obj_vel.linear.x - ego_vel.linear.x if hasattr(obj_vel, 'linear') else 0.0,
            obj_vel.linear.y - ego_vel.linear.y if hasattr(obj_vel, 'linear') else 0.0
        ])

        rel_dist = np.linalg.norm(rel_pos)
        if rel_dist < 0.01:
            return 0.0

        rel_speed = -np.dot(rel_pos, rel_vel) / rel_dist
        if rel_speed <= 0:
            return INF

        return rel_dist / rel_speed

    def check_collisions(self):
        if self.ego_pose is None:
            return

        min_ttc = INF
        ego_pos = self.ego_pose.position
        ego_yaw = self.get_yaw_from_quaternion(self.ego_pose.orientation)
        ego_pos_np = np.array([ego_pos.x, ego_pos.y])

        for obj in self.objects:
            obj_pos = np.array([obj.position.x, obj.position.y])
            rel_pos = obj_pos - ego_pos_np

            rel_heading = math.atan2(rel_pos[1], rel_pos[0]) - ego_yaw
            if abs(rel_heading) > math.pi / 2:
                continue

            ttc = self.compute_ttc(self.ego_pose, self.ego_velocity, obj)
            if ttc < min_ttc:
                min_ttc = ttc

        level = CollisionLevel.SAFE
        brake_cmd = Twist()

        if min_ttc < self.ttc_full_brake:
            level = CollisionLevel.CRITICAL
            brake_cmd.linear.x = 0.0
            brake_cmd.angular.z = 0.0
            self.aeb_cmd_pub.publish(brake_cmd)
            self.get_logger().error(f'AEB Level 3: 全力制动 (TTC={min_ttc:.2f}s)')
        elif min_ttc < self.ttc_brake:
            level = CollisionLevel.CRITICAL
            brake_cmd.linear.x = -self.ego_velocity * 0.6
            self.aeb_cmd_pub.publish(brake_cmd)
            self.get_logger().warn(f'AEB Level 2: 部分制动 (TTC={min_ttc:.2f}s)')
        elif min_ttc < self.ttc_warning:
            level = CollisionLevel.WARNING
            self.get_logger().info(f'碰撞预警 (TTC={min_ttc:.2f}s)')

        warning_msg = Bool()
        warning_msg.data = (level != CollisionLevel.SAFE)
        self.collision_warning_pub.publish(warning_msg)

        level_msg = String()
        level_msg.data = level.name
        self.collision_level_pub.publish(level_msg)

        self.publish_visualization(min_ttc, level)

    def check_deviation(self):
        if self.ego_pose is None or self.planned_trajectory is None:
            return

        if not self.planned_trajectory.poses:
            return

        nearest_waypoint = self.find_nearest_waypoint(
            self.ego_pose, self.planned_trajectory.poses)

        lateral_offset = self.compute_lateral_offset(
            self.ego_pose, nearest_waypoint)
        heading_error = self.compute_heading_error(
            self.ego_pose, nearest_waypoint)

        max_offset = self.lane_width * self.deviation_threshold
        deviation = abs(lateral_offset)
        deviation_msg = Float32()
        deviation_msg.data = deviation
        self.deviation_pub.publish(deviation_msg)

        now = self.get_clock().now()
        dt = (now - self.last_deviation_time).nanoseconds / 1e9
        if dt > 0 and deviation > max_offset:
            self.cumulative_deviation += deviation * dt
        else:
            self.cumulative_deviation = max(0.0, self.cumulative_deviation - dt * 0.5)
        self.last_deviation_time = now

        if deviation > max_offset:
            self.get_logger().warn(
                f'车道偏离: lateral={lateral_offset:.3f}m, '
                f'heading={math.degrees(heading_error):.1f}deg')

        if self.cumulative_deviation > self.lane_width * 2.0:
            self.get_logger().error('严重车道偏离，建议驾驶员接管')

    def find_nearest_waypoint(self, ego_pose, waypoints):
        ego_pos = np.array([
            ego_pose.position.x,
            ego_pose.position.y
        ])
        min_dist = INF
        nearest = waypoints[0]

        for wp in waypoints:
            wp_pos = np.array([
                wp.pose.position.x,
                wp.pose.position.y
            ])
            dist = np.linalg.norm(ego_pos - wp_pos)
            if dist < min_dist:
                min_dist = dist
                nearest = wp

        return nearest

    def compute_lateral_offset(self, ego_pose, waypoint):
        ego_pos = np.array([ego_pose.position.x, ego_pose.position.y])
        wp_pos = np.array([
            waypoint.pose.position.x,
            waypoint.pose.position.y
        ])
        wp_yaw = self.get_yaw_from_quaternion(waypoint.pose.orientation)

        vec_to_ego = ego_pos - wp_pos
        normal = np.array([-math.sin(wp_yaw), math.cos(wp_yaw)])
        lateral_offset = np.dot(vec_to_ego, normal)

        return lateral_offset

    def compute_heading_error(self, ego_pose, waypoint):
        ego_yaw = self.get_yaw_from_quaternion(ego_pose.orientation)
        wp_yaw = self.get_yaw_from_quaternion(waypoint.pose.orientation)
        error = ego_yaw - wp_yaw
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        return error

    def execute_emergency_brake(self):
        brake_cmd = Twist()
        brake_cmd.linear.x = 0.0
        brake_cmd.angular.z = 0.0
        self.aeb_cmd_pub.publish(brake_cmd)
        self.get_logger().warn('执行紧急制动')

    def publish_visualization(self, min_ttc, level):
        marker_array = MarkerArray()

        ttc_marker = Marker()
        ttc_marker.header.frame_id = 'map'
        ttc_marker.ns = 'ttc'
        ttc_marker.id = 0
        ttc_marker.type = Marker.TEXT_VIEW_FACING
        ttc_marker.pose.position.x = self.ego_pose.position.x
        ttc_marker.pose.position.y = self.ego_pose.position.y
        ttc_marker.pose.position.z = 3.0
        ttc_marker.scale.z = 0.8
        ttc_marker.text = f'TTC: {min_ttc:.1f}s [{level.name}]'

        if level == CollisionLevel.CRITICAL:
            ttc_marker.color.r = 1.0
            ttc_marker.color.g = 0.0
            ttc_marker.color.b = 0.0
        elif level == CollisionLevel.WARNING:
            ttc_marker.color.r = 1.0
            ttc_marker.color.g = 1.0
            ttc_marker.color.b = 0.0
        else:
            ttc_marker.color.r = 0.0
            ttc_marker.color.g = 1.0
            ttc_marker.color.b = 0.0
        ttc_marker.color.a = 1.0

        marker_array.markers.append(ttc_marker)
        self.visualization_pub.publish(marker_array)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def cleanup(self):
        brake_cmd = Twist()
        brake_cmd.linear.x = 0.0
        self.aeb_cmd_pub.publish(brake_cmd)
        self.get_logger().info('SafetyMonitor cleanup: 车辆已停止')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
