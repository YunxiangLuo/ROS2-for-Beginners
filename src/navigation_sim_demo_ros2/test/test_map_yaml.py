import unittest
from pathlib import Path

import yaml


class NavigationMapYamlTest(unittest.TestCase):
    def test_map_yaml_matches_expected_asset(self):
        package_root = Path(__file__).resolve().parents[1]
        map_yaml = package_root / "maps" / "Software_Museum.yaml"
        with map_yaml.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)

        self.assertEqual(data["image"], "Software_Museum.pgm")
        self.assertAlmostEqual(float(data["resolution"]), 0.05)
        self.assertEqual(len(data["origin"]), 3)
