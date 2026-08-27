import asyncio
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
from action_demo_interfaces.action import DoDishes
from rclpy.parameter import Parameter

from action_demo.dishes_client import DoDishesClient
from action_demo.dishes_server import DoDishesServer


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_action_interface_uses_repository_field_names():
    goal = DoDishes.Goal()
    result = DoDishes.Result()
    feedback = DoDishes.Feedback()

    goal.dishwasher_id = 2
    result.total_dishes_cleaned = 10
    feedback.percent_complete = 20.0

    assert goal.dishwasher_id == 2
    assert result.total_dishes_cleaned == 10
    assert feedback.percent_complete == pytest.approx(20.0)


def test_server_execute_reports_progress_and_result_without_waiting():
    node = DoDishesServer()
    node.set_parameters([
        Parameter('step_duration_sec', Parameter.Type.DOUBLE, 0.0),
    ])
    goal = DoDishes.Goal()
    goal.dishwasher_id = 2
    goal_handle = SimpleNamespace(
        request=goal,
        is_cancel_requested=False,
        publish_feedback=Mock(),
        succeed=Mock(),
        canceled=Mock(),
    )

    try:
        result = asyncio.run(node.execute(goal_handle))
        assert result.total_dishes_cleaned == 10
        assert goal_handle.publish_feedback.call_count == 5
        assert goal_handle.publish_feedback.call_args.args[0].percent_complete == pytest.approx(100.0)
        goal_handle.succeed.assert_called_once_with()
    finally:
        node.destroy_node()


def test_client_builds_existing_action_goal():
    node = DoDishesClient()
    try:
        goal = DoDishes.Goal()
        goal.dishwasher_id = 7
        assert hasattr(goal, 'dishwasher_id')
        assert not hasattr(goal, 'total_dishes')
    finally:
        node.destroy_node()
