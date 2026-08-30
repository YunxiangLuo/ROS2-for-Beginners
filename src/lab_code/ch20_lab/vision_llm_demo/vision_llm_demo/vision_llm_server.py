"""Wrap a vision-LLM caption backend behind a std_srvs/Trigger service."""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from .vision_provider import VisionProvider


class VisionLLMServer(Node):

    def __init__(self):
        super().__init__('vision_llm_server')
        self.declare_parameter('provider', 'mock')
        self.declare_parameter('prompt', '')
        self.declare_parameter('image_path', '')
        provider = self.get_parameter('provider').value
        prompt = self.get_parameter('prompt').value
        image_path = self.get_parameter('image_path').value
        self.vision = VisionProvider(
            provider=provider, prompt=prompt, image_path=image_path)
        self.service = self.create_service(
            Trigger, 'vision_llm', self.on_trigger)
        self.get_logger().info(
            'vision_llm service ready (provider=%s)' % provider)

    def on_trigger(self, request, response):
        caption = self.vision.caption()
        response.success = True
        response.message = caption
        self.get_logger().info('caption: %s' % caption)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VisionLLMServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
