import threading
import time
import unittest

from geometry_msgs.msg import TransformStamped, Twist
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from tf_follower_ros2.tf_follower import TfFollowerNode


class FakeTransformPublisher(Node):
    def __init__(self) -> None:
        super().__init__("fake_transform_publisher_test")
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.publish_transform)

    def publish_transform(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "mybot_link"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = 3.0
        transform.transform.translation.y = 1.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(transform)


class TwistCollector(Node):
    def __init__(self) -> None:
        super().__init__("twist_collector_test")
        self.messages = []
        self.subscription = self.create_subscription(Twist, "/mybot_cmd_vel", self.callback, 10)

    def callback(self, message: Twist) -> None:
        self.messages.append(message)


class TfFollowerIntegrationTest(unittest.TestCase):
    def test_follower_publishes_twist_from_tf(self):
        rclpy.init()
        follower = TfFollowerNode()
        broadcaster = FakeTransformPublisher()
        collector = TwistCollector()

        executor = MultiThreadedExecutor()
        executor.add_node(follower)
        executor.add_node(broadcaster)
        executor.add_node(collector)

        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        deadline = time.time() + 3.0
        try:
            while time.time() < deadline:
                if collector.messages:
                    break
                time.sleep(0.05)

            self.assertTrue(collector.messages, "Expected tf_follower to publish at least one Twist")
            command = collector.messages[-1]
            self.assertGreater(command.linear.x, 0.0)
            self.assertNotEqual(command.angular.z, 0.0)
        finally:
            executor.shutdown()
            spin_thread.join(timeout=1.0)
            follower.destroy_node()
            broadcaster.destroy_node()
            collector.destroy_node()
            rclpy.shutdown()
