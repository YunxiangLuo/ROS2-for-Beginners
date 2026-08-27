import rclpy
from rclpy.node import Node


class ParamDemoNode(Node):
    def __init__(self) -> None:
        super().__init__("param_demo_py")
        self.loop_count = 0
        self.declare_parameter("param1", 111)
        self.declare_parameter("param2", 222)
        self.declare_parameter("param3", 33333)
        self.declare_parameter("param4", 0)
        self.declare_parameter("param5", 0)
        self.declare_parameter("max_loops", 3)

        self.get_logger().info(f"Get param1 = {self.get_parameter('param1').value}")
        self.get_logger().info(f"Get param2 = {self.get_parameter('param2').value}")
        self.get_logger().info(f"Get param3 = {self.get_parameter('param3').value}")

        self.set_parameters(
            [
                rclpy.parameter.Parameter("param4", value=4),
                rclpy.parameter.Parameter("param5", value=5),
            ]
        )
        self.timer = self.create_timer(1.0, self.tick)

    def tick(self) -> None:
        self.loop_count += 1
        max_loops = int(self.get_parameter("max_loops").value)
        if self.loop_count == 1:
            self.set_parameters([rclpy.parameter.Parameter("param2", value=2)])
            self.get_logger().info("Param2 updated to 2")

        if self.loop_count == 2 and self.has_parameter("param5"):
            self.undeclare_parameter("param5")
            self.get_logger().info("Param5 deleted via undeclare_parameter")

        names = sorted(self._parameters.keys())
        self.get_logger().info("=============Loop==============")
        self.get_logger().info(f"param list: {names}")
        for name in names:
            self.get_logger().info(f"parameter {name} = {self.get_parameter(name).value}")
        if self.loop_count >= max_loops:
            self.get_logger().info(f"Reached max_loops={max_loops}, shutting down")
            self.destroy_timer(self.timer)
            self.context.shutdown()


def main() -> None:
    rclpy.init()
    node = ParamDemoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
