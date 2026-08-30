"""Call the /vision_llm Trigger service ``count`` times and log captions."""

import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class VisionLLMClient(Node):

    def __init__(self):
        super().__init__('vision_llm_client')
        self.declare_parameter('count', 1)
        self.declare_parameter('interval_sec', 0.5)
        self.count = max(1, int(self.get_parameter('count').value))
        self.interval = max(
            0.0, float(self.get_parameter('interval_sec').value))
        self.client = self.create_client(Trigger, 'vision_llm')

    def run(self):
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('service /vision_llm is unavailable')
            return 1
        for index in range(self.count):
            future = self.client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            if future.result() is None:
                self.get_logger().error('call %d timed out' % (index + 1))
                return 1
            self.get_logger().info('[%d/%d] %s' % (
                index + 1, self.count, future.result().message))
            if self.interval > 0.0 and index + 1 < self.count:
                time.sleep(self.interval)
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = VisionLLMClient()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
