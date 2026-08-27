import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from nav2_msgs.msg import Costmap


class NavMonitor(Node):
    def __init__(self):
        super().__init__('nav_monitor')
        self.create_subscription(Path, '/plan', self.plan_cb, 10)
        self.create_subscription(Path, '/local_plan', self.local_plan_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.plan_count = 0
        self.last_plan_key = None

    def plan_cb(self, msg):
        if not msg.poses:
            return
        key = (msg.poses[0].pose.position.x, msg.poses[0].pose.position.y,
               msg.poses[-1].pose.position.x, msg.poses[-1].pose.position.y)
        if key != self.last_plan_key:
            self.plan_count += 1
            self.get_logger().info(f'New global plan #{self.plan_count}')
            self.last_plan_key = key

    def local_plan_cb(self, msg):
        self.get_logger().info(f'Local plan: {len(msg.poses)} poses')

    def cmd_cb(self, msg):
        self.get_logger().info(
            f'Cmd vel: linear={msg.linear.x:.2f} angular={msg.angular.z:.2f}'
        )

    def odom_cb(self, msg):
        pass


def main():
    rclpy.init()
    node = NavMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
