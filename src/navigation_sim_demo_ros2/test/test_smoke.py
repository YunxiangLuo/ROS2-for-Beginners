import unittest
from pathlib import Path

from navigation_sim_demo_ros2 import initial_pose_publisher, nav_goal_runner


class NavigationSimSmokeTest(unittest.TestCase):
    def test_modules_import(self):
        self.assertIsNotNone(initial_pose_publisher)
        self.assertIsNotNone(nav_goal_runner)

    def test_assets_exist(self):
        package_root = Path(__file__).resolve().parents[1]
        self.assertTrue((package_root / "launch" / "nav2_demo.launch.py").is_file())
        self.assertTrue((package_root / "maps" / "Software_Museum.yaml").is_file())
        self.assertTrue((package_root / "maps" / "Software_Museum.pgm").is_file())
        self.assertTrue((package_root / "params" / "nav2_params.yaml").is_file())
        self.assertTrue((package_root / "rviz" / "navigation.rviz").is_file())
