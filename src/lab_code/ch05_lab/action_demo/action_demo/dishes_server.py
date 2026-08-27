"""DoDishes Action Server — 模拟洗碗任务，支持进度反馈和取消"""
import asyncio
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from action_demo_interfaces.action import DoDishes


class DoDishesServer(Node):
    def __init__(self):
        super().__init__('do_dishes_server')
        self.declare_parameter('step_duration_sec', 1.0)
        self.action_server = ActionServer(
            self, DoDishes, 'do_dishes',
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        self.get_logger().info(f'收到目标: 洗碗机 {goal_request.dishwasher_id}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求')
        return CancelResponse.ACCEPT

    async def execute(self, goal_handle):
        dishwasher_id = goal_handle.request.dishwasher_id
        duration = float(self.get_parameter('step_duration_sec').value)
        fb = DoDishes.Feedback()
        for step in range(1, 6):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = DoDishes.Result()
                result.total_dishes_cleaned = (step - 1) * dishwasher_id
                return result
            await asyncio.sleep(duration)
            fb.percent_complete = float(step * 20)
            goal_handle.publish_feedback(fb)
            self.get_logger().info(f'进度: {fb.percent_complete:.0f}%')
        goal_handle.succeed()
        result = DoDishes.Result()
        result.total_dishes_cleaned = dishwasher_id * 5
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DoDishesServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
