import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    """服务客户端 — 带超时和重试"""

    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

    def call(self, a, b, timeout_sec=5.0, max_retries=3):
        for i in range(max_retries):
            if self.client.wait_for_service(timeout_sec=2.0):
                req = AddTwoInts.Request()
                req.a = a
                req.b = b
                future = self.client.call_async(req)
                rclpy.spin_until_future_complete(
                    self, future, timeout_sec=timeout_sec)
                if future.result() is not None:
                    return future.result().sum
            self.get_logger().warn(f'重试 {i+1}/{max_retries}')
        return None


def main(args=None):
    rclpy.init(args=args)
    client = AddTwoIntsClient()
    a = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    b = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    result = client.call(a, b)
    if result is not None:
        client.get_logger().info(f'结果: {result}')
    else:
        client.get_logger().error('调用失败')
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
