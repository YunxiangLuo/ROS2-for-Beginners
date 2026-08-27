"""参数系统演示节点 — 声明、读取、动态设置参数，含回调验证"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


class ParamDemoNode(Node):
    def __init__(self):
        super().__init__('param_demo')

        # 声明参数: 名称, 默认值, 描述
        self.declare_parameter('robot_name', 'xbot')
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('mode', 'auto')
        self.declare_parameter('enable_debug', False)

        # 注册参数变化回调（验证新值合法性）
        self.add_on_set_parameters_callback(self.validate_params)

        # 每秒输出当前参数值
        self.timer = self.create_timer(1.0, self.print_params)

    def validate_params(self, params):
        """验证参数合法性 — 非法参数将被拒绝"""
        for param in params:
            if param.name == 'max_speed':
                if param.value < 0.0 or param.value > 10.0:
                    return SetParametersResult(
                        successful=False,
                        reason='max_speed 必须在 [0.0, 10.0] 范围内')
            elif param.name == 'mode':
                if param.value not in ('auto', 'manual', 'hybrid'):
                    return SetParametersResult(
                        successful=False,
                        reason="mode 必须是 'auto', 'manual' 或 'hybrid'")
        return SetParametersResult(successful=True)

    def print_params(self):
        name = self.get_parameter('robot_name').value
        speed = self.get_parameter('max_speed').value
        mode = self.get_parameter('mode').value
        debug = self.get_parameter('enable_debug').value
        self.get_logger().info(
            f'Robot: {name} | Speed: {speed}m/s | '
            f'Mode: {mode} | Debug: {debug}')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ParamDemoNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
