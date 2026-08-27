"""Recipe validation service with an explicit deterministic fallback."""

import json

from course_lab_interfaces.srv import ValidateRecipe
import rclpy
from rclpy.node import Node


def build_recipe_prompt(recipe: str) -> str:
    return (
        "Validate this school chemistry recipe for ratio, products, and safety. "
        "Return JSON with is_valid, feedback, products, and safety_warnings.\n\n"
        f"{recipe}"
    )


def parse_validation_response(text: str) -> dict:
    start = text.find("{")
    end = text.rfind("}")
    if start < 0 or end < start:
        raise ValueError("Validation response does not contain a JSON object")
    result = json.loads(text[start : end + 1])
    if not isinstance(result.get("is_valid"), bool):
        raise ValueError("Validation response requires a boolean is_valid field")
    return result


class RecipeValidator(Node):
    def __init__(self):
        super().__init__("recipe_validator")
        self.declare_parameter("simulation_mode", True)
        self.create_service(ValidateRecipe, "validate_recipe", self.validate_callback)

    def validate_callback(self, request, response):
        try:
            result = parse_validation_response(
                self.call_llm(build_recipe_prompt(request.recipe_text))
            )
        except (ValueError, json.JSONDecodeError) as error:
            result = {
                "is_valid": False,
                "feedback": f"Invalid validator response: {error}",
                "products": [],
                "safety_warnings": [],
            }
        response.success = result["is_valid"]
        response.message = json.dumps(result, ensure_ascii=False)
        return response

    def call_llm(self, prompt: str) -> str:
        if bool(self.get_parameter("simulation_mode").value):
            return json.dumps(
                {
                    "is_valid": True,
                    "feedback": "Simulation validation passed",
                    "products": [],
                    "safety_warnings": ["Manual safety review is still required"],
                }
            )
        from openai import OpenAI

        response = OpenAI().chat.completions.create(
            model="gpt-4o-mini",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.1,
        )
        return response.choices[0].message.content


def main(args=None):
    rclpy.init(args=args)
    node = RecipeValidator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
