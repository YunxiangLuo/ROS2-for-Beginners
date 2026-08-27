"""Lifecycle node that publishes a small forward velocity when active."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy


class HelloRos2Node(LifecycleNode):
    """Publish ``/cmd_vel`` only after the lifecycle node is activated."""

    def __init__(self):
        super().__init__('hello_ros2_lifecycle')
        self.declare_parameter('autostart', False)

        self.pub = None
        self.timer = None
        self.count = 0
        self.active = False
        self.get_logger().info(
            'Lifecycle 节点已创建，等待 configure；可设置 autostart:=true。')

    def on_configure(self, state):
        del state
        cmd_vel_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_lifecycle_publisher(
            Twist, '/cmd_vel', cmd_vel_qos)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info(
            'on_configure: LifecyclePublisher 和定时器创建完成。')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        result = super().on_activate(state)
        if result == TransitionCallbackReturn.SUCCESS:
            self.active = True
            self.get_logger().info('on_activate: 节点已激活。')
        return result

    def on_deactivate(self, state):
        self._publish_zero()
        result = super().on_deactivate(state)
        self.active = False
        return result

    def on_cleanup(self, state):
        del state
        self.active = False
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
        if self.pub is not None:
            self.destroy_publisher(self.pub)
            self.pub = None
        self.count = 0
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self._publish_zero()
        self.active = False
        return TransitionCallbackReturn.SUCCESS

    def _publish_zero(self):
        if self.pub is not None and self.active:
            msg = Twist()
            self.pub.publish(msg)

    def timer_callback(self):
        if not self.active or self.pub is None:
            return
        msg = Twist()
        msg.linear.x = 0.1
        self.pub.publish(msg)
        self.count += 1
        self.get_logger().info(
            f'第 {self.count} 次发布 /cmd_vel: '
            f'linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')

    def autostart(self):
        """Apply the configure and activate transitions when requested."""
        if not self.get_parameter('autostart').value:
            return False
        configured = self.trigger_configure()
        if configured != TransitionCallbackReturn.SUCCESS:
            return False
        return self.trigger_activate() == TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    node = HelloRos2Node()
    try:
        node.autostart()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断，节点退出。')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
