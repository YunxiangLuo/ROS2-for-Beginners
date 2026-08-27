import unittest

from navigation_sim_demo_ros2 import nav2_lifecycle_runner


class NavigationLifecycleRunnerSmokeTest(unittest.TestCase):
    def test_module_import(self):
        self.assertIsNotNone(nav2_lifecycle_runner)
