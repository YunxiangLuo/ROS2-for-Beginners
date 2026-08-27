import unittest

from slam_sim_demo_ros2 import slam_save_reload_runner


class SlamSaveReloadRunnerSmokeTest(unittest.TestCase):
    def test_module_import(self):
        self.assertIsNotNone(slam_save_reload_runner)
