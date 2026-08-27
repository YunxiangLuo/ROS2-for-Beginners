import unittest
from pathlib import Path


class UrdfDemoSmokeTest(unittest.TestCase):
    def test_urdf_demo_assets_exist(self):
        package_root = Path(__file__).resolve().parents[1]
        self.assertTrue((package_root / "launch" / "display_xacro.launch.py").is_file())
        self.assertTrue((package_root / "rviz" / "mybot_link.rviz").is_file())
        self.assertTrue((package_root / "urdf" / "mybot.xacro").is_file())
