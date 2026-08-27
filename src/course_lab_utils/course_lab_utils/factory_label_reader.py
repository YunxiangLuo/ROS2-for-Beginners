"""Label-reading service with VLM and deterministic simulation modes."""

import base64
import json

import cv2
from course_lab_interfaces.srv import ReadLabel
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def label_matches(expected: str, detected: str) -> bool:
    return expected.strip().casefold() in detected.strip().casefold()


class LabelReader(Node):
    def __init__(self):
        super().__init__("label_reader")
        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("simulated_label", "HCl")
        self.bridge = CvBridge()
        self.latest_image = None
        self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.create_service(ReadLabel, "read_label", self.read_label_callback)

    def image_callback(self, message):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(message, "bgr8")
        except CvBridgeError as error:
            self.get_logger().error(str(error))

    def read_label_callback(self, request, response):
        simulation = bool(self.get_parameter("simulation_mode").value)
        if self.latest_image is None and not simulation:
            response.success = False
            response.message = "No camera image is available"
            return response
        detected = self.call_vlm(self.latest_image)
        matches = label_matches(request.expected, detected)
        response.success = matches
        response.message = json.dumps(
            {
                "label_text": detected,
                "expected": request.expected,
                "matches": matches,
                "confidence": 0.95 if matches else 0.1,
            },
            ensure_ascii=False,
        )
        return response

    def call_vlm(self, image) -> str:
        if bool(self.get_parameter("simulation_mode").value):
            return str(self.get_parameter("simulated_label").value)
        success, encoded = cv2.imencode(".jpg", image)
        if not success:
            raise RuntimeError("Could not encode camera image")
        image_data = base64.b64encode(encoded).decode("ascii")
        from openai import OpenAI

        response = OpenAI().chat.completions.create(
            model="gpt-4o",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": "Read only the bottle label text."},
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{image_data}"},
                        },
                    ],
                }
            ],
            max_tokens=50,
        )
        return response.choices[0].message.content.strip()


def main(args=None):
    rclpy.init(args=args)
    node = LabelReader()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
