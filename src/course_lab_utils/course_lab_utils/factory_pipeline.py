"""Action server that orchestrates the deterministic factory lab workflow."""

import asyncio
import json

from course_lab_interfaces.action import Pipeline
from course_lab_interfaces.srv import ValidateRecipe
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node


DEFAULT_COMPONENTS = [
    {"name": "HCl", "volume_ml": 5.0},
    {"name": "NaOH", "volume_ml": 5.0},
    {"name": "Phenolphthalein", "volume_ml": 2.0},
]


def parse_recipe_components(recipe_text: str) -> list[dict]:
    try:
        parsed = json.loads(recipe_text)
    except json.JSONDecodeError:
        return [dict(component) for component in DEFAULT_COMPONENTS]
    components = parsed.get("components", []) if isinstance(parsed, dict) else []
    normalized = []
    for component in components:
        if not isinstance(component, dict) or not str(component.get("name", "")).strip():
            continue
        normalized.append(
            {
                "name": str(component["name"]),
                "volume_ml": float(component.get("volume_ml", 0.0)),
            }
        )
    return normalized or [dict(component) for component in DEFAULT_COMPONENTS]


class ExperimentPipeline(Node):
    TOTAL_STEPS = 6

    def __init__(self):
        super().__init__("experiment_pipeline")
        self.validator = self.create_client(ValidateRecipe, "validate_recipe")
        self.action_server = ActionServer(
            self,
            Pipeline,
            "run_experiment",
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def goal_callback(self, goal):
        if not goal.recipe_text.strip():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        del goal_handle
        return CancelResponse.ACCEPT

    async def execute(self, goal_handle):
        result = Pipeline.Result()
        self._publish_feedback(goal_handle, 1, "validate recipe")
        validation = await self.validate_recipe(goal_handle.request.recipe_text)
        if validation is None or not validation.success:
            goal_handle.abort()
            result.message = "Recipe validation failed"
            return result

        self._publish_feedback(goal_handle, 2, "parse components")
        components = parse_recipe_components(goal_handle.request.recipe_text)
        await asyncio.sleep(0.05)
        for component in components:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.message = "Experiment canceled"
                return result
            self._publish_feedback(goal_handle, 3, f"detect {component['name']}")
            await asyncio.sleep(0.05)
            self._publish_feedback(goal_handle, 4, f"verify {component['name']} label")
            await asyncio.sleep(0.05)
            self._publish_feedback(goal_handle, 5, f"transfer {component['name']}")
            await asyncio.sleep(0.05)

        self._publish_feedback(goal_handle, 6, "complete")
        goal_handle.succeed()
        result.success = True
        result.message = f"Processed {len(components)} component(s)"
        return result

    async def validate_recipe(self, recipe_text):
        if not self.validator.wait_for_service(timeout_sec=2.0):
            return None
        request = ValidateRecipe.Request()
        request.recipe_text = recipe_text
        return await self.validator.call_async(request)

    def _publish_feedback(self, goal_handle, current_step, step_name):
        feedback = Pipeline.Feedback()
        feedback.current_step = current_step
        feedback.total_steps = self.TOTAL_STEPS
        feedback.step_name = step_name
        goal_handle.publish_feedback(feedback)

    def destroy_node(self):
        self.action_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentPipeline()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
