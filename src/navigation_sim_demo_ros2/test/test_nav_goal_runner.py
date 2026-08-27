import unittest

from navigation_sim_demo_ros2 import nav_goal_runner


class NavigationGoalRunnerSmokeTest(unittest.TestCase):
    def test_module_import(self):
        self.assertIsNotNone(nav_goal_runner)
