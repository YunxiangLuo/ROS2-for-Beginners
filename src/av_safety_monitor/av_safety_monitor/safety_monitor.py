import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path

try:
    from av_carla_interfaces.msg import CollisionEvent
except ImportError:
    CollisionEvent = None

try:
    from av_carla_interfaces.msg import PerceptionObjectArray
except ImportError:
    PerceptionObjectArray = None


class AlertLevel:
    WARNING = 1
    CRITICAL = 2
    EMERGENCY = 3


LEVEL_NAMES = {1: 'WARNING', 2: 'CRITICAL', 3: 'EMERGENCY'}
LEVEL_COLORS = {
    1: (1.0, 1.0, 0.0),
    2: (1.0, 0.65, 0.0),
    3: (1.0, 0.0, 0.0),
}


class SafetyMonitor(Node):

    def __init__(self):
        super().__init__('safety_monitor')

        self.declare_parameter('ttc_threshold_warning', 4.0)
        self.declare_parameter('ttc_threshold_critical', 2.5)
        self.declare_parameter('ttc_threshold_emergency', 1.5)
        self.declare_parameter('lane_deviation_warning', 0.5)
        self.declare_parameter('lane_deviation_critical', 1.0)
        self.declare_parameter('emergency_brake_decel', -5.0)

        self.ttc_warn = self.get_parameter('ttc_threshold_warning').value
        self.ttc_crit = self.get_parameter('ttc_threshold_critical').value
        self.ttc_emerg = self.get_parameter('ttc_threshold_emergency').value
        self.lane_warn = self.get_parameter('lane_deviation_warning').value
        self.lane_crit = self.get_parameter('lane_deviation_critical').value
        self.brake_decel = self.get_parameter('emergency_brake_decel').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self._collision_sub = self.create_subscription(
            Point if CollisionEvent is None else CollisionEvent,
            '/carla/ego_vehicle/collision',
            self._collision_callback,
            qos,
        )
        self._plan_sub = self.create_subscription(
            Path,
            '/plan',
            self._plan_callback,
            qos,
        )
        self._ego_state_sub = self.create_subscription(
            TwistStamped,
            '/ego_state',
            self._ego_state_callback,
            qos,
        )
        self._perception_sub = None
        if PerceptionObjectArray is not None:
            self._perception_sub = self.create_subscription(
                PerceptionObjectArray,
                '/perception_objects',
                self._perception_callback,
                qos,
            )

        self._status_pub = self.create_publisher(String, '/safety_status', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/safety_markers', 10)
        self._emergency_pub = self.create_publisher(Bool, '/emergency_stop', 10)

        self._current_alert = AlertLevel.WARNING
        self._ego_speed = 0.0
        self._plan = None
        self._nearest_obstacle_distance = float('inf')
        self._nearest_obstacle_velocity = 0.0
        self._collision_reported = False

        self._timer = self.create_timer(0.1, self._monitor_loop)
        self.get_logger().info('SafetyMonitor initialized')

    def _collision_callback(self, msg):
        self._collision_reported = True
        self.get_logger().warn('Collision detected!')

    def _plan_callback(self, msg):
        self._plan = msg

    def _ego_state_callback(self, msg):
        self._ego_speed = math.sqrt(
            msg.twist.linear.x ** 2 +
            msg.twist.linear.y ** 2 +
            msg.twist.linear.z ** 2
        )

    def _perception_callback(self, msg):
        """更新最近障碍物距离与速度(障碍位于自车坐标系, 取平面距离)。"""
        nearest_dist = float('inf')
        nearest_vel = 0.0
        for obj in getattr(msg, 'objects', []):
            pos = getattr(obj.pose, 'position', None)
            if pos is None:
                continue
            dist = math.hypot(pos.x, pos.y)
            if dist < nearest_dist:
                nearest_dist = dist
                vel = getattr(obj, 'velocity', None)
                if vel is not None:
                    nearest_vel = math.sqrt(vel.x ** 2 + vel.y ** 2)
                else:
                    nearest_vel = 0.0
        self._nearest_obstacle_distance = nearest_dist
        self._nearest_obstacle_velocity = nearest_vel

    def _compute_ttc(self):
        relative_speed = self._ego_speed - self._nearest_obstacle_velocity
        if relative_speed <= 0.0:
            return float('inf')
        if self._nearest_obstacle_distance <= 0.0:
            return 0.0
        return self._nearest_obstacle_distance / relative_speed

    def _compute_lane_deviation(self):
        if self._plan is None or not self._plan.poses:
            return 0.0
        return 0.0

    def _publish_markers(self, alert_level):
        marker_array = MarkerArray()

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'safety_alert'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        r, g, b = LEVEL_COLORS.get(alert_level, (1.0, 1.0, 1.0))
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.8
        marker.lifetime.sec = 1
        marker_array.markers.append(marker)

        self._marker_pub.publish(marker_array)

    def _monitor_loop(self):
        ttc = self._compute_ttc()
        lane_dev = self._compute_lane_deviation()

        alert_level = AlertLevel.WARNING
        alert_reasons = []

        if ttc < self.ttc_emerg:
            alert_level = AlertLevel.EMERGENCY
            alert_reasons.append(f'TTC={ttc:.2f}s EMERGENCY')
        elif ttc < self.ttc_crit:
            alert_level = AlertLevel.CRITICAL
            alert_reasons.append(f'TTC={ttc:.2f}s CRITICAL')
        elif ttc < self.ttc_warn:
            alert_level = max(alert_level, AlertLevel.WARNING)
            alert_reasons.append(f'TTC={ttc:.2f}s WARNING')

        if lane_dev > self.lane_crit:
            alert_level = AlertLevel.EMERGENCY
            alert_reasons.append(f'Lane deviation={lane_dev:.2f}m EMERGENCY')
        elif lane_dev > self.lane_warn:
            alert_level = max(alert_level, AlertLevel.CRITICAL)
            alert_reasons.append(f'Lane deviation={lane_dev:.2f}m WARNING')

        if self._collision_reported:
            alert_level = AlertLevel.EMERGENCY
            alert_reasons.append('Collision detected')
            self._collision_reported = False

        if alert_level == AlertLevel.EMERGENCY:
            emergency_msg = Bool()
            emergency_msg.data = True
            self._emergency_pub.publish(emergency_msg)

        status_msg = String()
        status_msg.data = f'[{LEVEL_NAMES[alert_level]}] ' + ' | '.join(alert_reasons) if alert_reasons else f'[{LEVEL_NAMES[alert_level]}] Normal'
        self._status_pub.publish(status_msg)

        self._publish_markers(alert_level)

        if alert_level != self._current_alert:
            if alert_level >= AlertLevel.CRITICAL:
                self.get_logger().warn(
                    f'Safety status changed to {LEVEL_NAMES[alert_level]}: {status_msg.data}')
            else:
                self.get_logger().info(
                    f'Safety status changed to {LEVEL_NAMES[alert_level]}: {status_msg.data}')
            self._current_alert = alert_level


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

