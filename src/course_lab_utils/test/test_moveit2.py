from math import pi, sqrt
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from moveit_msgs.msg import RobotState as RobotStateMessage

from course_lab_utils import moveit2
from course_lab_utils import (
    aruco_pick_server,
    aruco_detection_node,
    color_detection_node,
    factory_arm_controller,
    factory_bottle_detector,
    factory_bottle_localizer,
    factory_label_reader,
    factory_pipeline,
    factory_recipe_validator,
    moveit_attach_demo,
    moveit_beeline_demo,
    moveit_obstacles_demo,
    moveit_pick_place_demo,
    target_publisher,
    vision_pickup_pipeline,
)


class MoveIt2HelpersTest(unittest.TestCase):
    def test_quaternion_from_euler(self):
        quaternion = moveit2.quaternion_from_euler(0.0, 0.0, pi / 2)
        self.assertAlmostEqual(quaternion[2], sqrt(0.5))
        self.assertAlmostEqual(quaternion[3], sqrt(0.5))

    @patch.object(moveit2, "RobotState")
    def test_set_joint_goal_builds_robot_state(self, robot_state_type):
        moveit = MagicMock()
        component = MagicMock()
        state = robot_state_type.return_value

        result = moveit2.set_joint_goal(moveit, component, "xarm", [0, 1])

        state.set_to_default_values.assert_called_once_with()
        group, values = state.set_joint_group_active_positions.call_args.args
        self.assertEqual(group, "xarm")
        self.assertEqual(values.tolist(), [0.0, 1.0])
        component.set_start_state_to_current_state.assert_called_once_with()
        component.set_goal_state.assert_called_once_with(robot_state=state)
        self.assertIs(result, state)

    def test_plan_and_execute_uses_moveit_instance(self):
        moveit = MagicMock()
        component = MagicMock()
        node = MagicMock()
        trajectory = object()
        component.plan.return_value = SimpleNamespace(trajectory=trajectory)
        with patch.object(moveit2, "ensure_execution_servers", return_value=True) as guard:
            self.assertTrue(moveit2.plan_and_execute(moveit, component, node))
        guard.assert_called_once_with(node)
        moveit.execute.assert_called_once_with(trajectory, controllers=[])

    def test_plan_and_execute_skips_execution_without_servers(self):
        moveit = MagicMock()
        component = MagicMock()
        component.plan.return_value = SimpleNamespace(trajectory=object())
        with patch.object(moveit2, "ensure_execution_servers", return_value=False):
            self.assertFalse(moveit2.plan_and_execute(moveit, component, MagicMock()))
        moveit.execute.assert_not_called()

    def test_ensure_execution_servers_fails_fast(self):
        node = MagicMock()
        with patch.object(moveit2, "ActionClient") as client_type:
            client = client_type.return_value
            client.wait_for_server.return_value = False
            self.assertFalse(moveit2.ensure_execution_servers(node, timeout_sec=0.1))
        self.assertEqual(client_type.call_count, 1)

    def test_ensure_execution_servers_waits_for_all(self):
        node = MagicMock()
        with patch.object(moveit2, "ActionClient") as client_type:
            client = client_type.return_value
            client.wait_for_server.return_value = True
            self.assertTrue(moveit2.ensure_execution_servers(node, timeout_sec=0.1))
        self.assertEqual(client_type.call_count, len(moveit2.EXECUTION_ACTION_SERVERS))

    def test_build_cartesian_request(self):
        start_state = RobotStateMessage()
        request = moveit2.build_cartesian_request(
            start_state,
            "base_link",
            "xarm",
            "tool_link",
            [],
        )
        self.assertEqual(request.header.frame_id, "base_link")
        self.assertEqual(request.group_name, "xarm")
        self.assertEqual(request.link_name, "tool_link")
        self.assertAlmostEqual(request.max_step, 0.01)
        self.assertTrue(request.avoid_collisions)

    def test_example_modules_import(self):
        for module in (
            moveit_attach_demo,
            aruco_pick_server,
            aruco_detection_node,
            color_detection_node,
            factory_arm_controller,
            factory_bottle_detector,
            factory_bottle_localizer,
            factory_label_reader,
            factory_pipeline,
            factory_recipe_validator,
            moveit_beeline_demo,
            moveit_obstacles_demo,
            moveit_pick_place_demo,
            target_publisher,
            vision_pickup_pipeline,
        ):
            self.assertTrue(callable(module.main))
