import unittest
from pathlib import Path

from slam_sim_demo_ros2 import slam_map_runner


class SlamSimSmokeTest(unittest.TestCase):
    def test_modules_import(self):
        self.assertIsNotNone(slam_map_runner)

    def test_assets_exist(self):
        package_root = Path(__file__).resolve().parents[1]
        self.assertTrue((package_root / "launch" / "slam_demo.launch.py").is_file())
        self.assertTrue((package_root / "launch" / "slam_depth_demo.launch.py").is_file())
        self.assertTrue((package_root / "params" / "slam_toolbox_params.yaml").is_file())
        self.assertTrue((package_root / "rviz" / "slam.rviz").is_file())
