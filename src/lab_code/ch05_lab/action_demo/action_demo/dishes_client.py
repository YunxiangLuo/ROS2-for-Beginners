"""DoDishes Action Client — 发送洗碗任务并接收进度反馈"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_demo_interfaces.action import DoDishes


class DoDishesClient(Node):
    def __init__(self):
        super().__init__('do_dishes_client')
        self.client = ActionClient(self, DoDishes, 'do_dishes')

    def send_goal(self, total_dishes):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('动作服务不可用')
            return False
        goal = DoDishes.Goal()
        goal.dishwasher_id = total_dishes
        self.client.send_goal_async(
            goal, feedback_callback=self.feedback_cb
        ).add_done_callback(self.goal_response_cb)
        return True

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('目标被拒绝')
            return
        self.get_logger().info('目标已接受')
        goal_handle.get_result_async().add_done_callback(self.result_cb)

    def feedback_cb(self, fb_msg):
        p = fb_msg.feedback.percent_complete
        self.get_logger().info(f'反馈: {p:.0f}%')

    def result_cb(self, future):
        r = future.result().result
        self.get_logger().info(
            f'完成: 清洗盘子总数={r.total_dishes_cleaned}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DoDishesClient()
    if not node.send_goal(2):
        node.destroy_node()
        rclpy.shutdown()
        return
    rclpy.spin(node)


if __name__ == '__main__':
    main()
