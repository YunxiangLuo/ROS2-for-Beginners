import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


def add_two_ints(a, b):
    """Return the deterministic result used by the service callback."""
    return a + b


class AddTwoIntsServer(Node):
    """两整数相加服务 — 处理加法请求"""

    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts, 'add_two_ints', self.handle_add)

    def handle_add(self, request, response):
        response.sum = add_two_ints(request.a, request.b)
        self.get_logger().info(f'{request.a}+{request.b}={response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
