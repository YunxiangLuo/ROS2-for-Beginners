import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanInjector(Node):
    def __init__(self):
        super().__init__('scan_injector')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)
        self.angle = 0.0

    def publish_scan(self):
        self.angle += 0.02
        scan = LaserScan()
        scan.header.frame_id = 'laser'
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0
        scan.time_increment = 1.0 / 360.0 / 180.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 3.5

        obstacle_x = 2.0 + 0.8 * math.sin(self.angle * 0.5)
        obstacle_y = 0.0 + 0.8 * math.cos(self.angle * 0.5)

        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        ranges = []
        for i in range(num_readings):
            theta = scan.angle_min + i * scan.angle_increment
            dx = obstacle_x - 0.0
            dy = obstacle_y - 0.0
            dist = math.sqrt(dx * dx + dy * dy)
            angle_to_obs = math.atan2(dy, dx)
            angle_diff = abs(theta - angle_to_obs)
            if angle_diff < 0.05 and dist < 3.0 and dist > 0.1:
                ranges.append(dist)
            else:
                ranges.append(3.5)
        scan.ranges = ranges
        scan.intensities = [0.0] * num_readings
        self.pub.publish(scan)


def main():
    rclpy.init()
    node = ScanInjector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
