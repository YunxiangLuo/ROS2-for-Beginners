"""Tests for slam_lab assets and AMCL configuration."""
import os
import xml.etree.ElementTree as ElementTree
from pathlib import Path

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_amcl_launch_defaults_to_navigation_sim_demo_map():
    text = (PACKAGE_ROOT / "launch" / "amcl_localization.launch.py").read_text()
    assert "Software_Museum.yaml" in text
    assert "navigation_sim_demo_ros2" in text
    assert "~/maps/lab10_map.yaml" not in text


def test_online_mapping_launch_exists():
    assert (PACKAGE_ROOT / "launch" / "online_mapping.launch.py").is_file()


def test_nav2_localization_yaml_uses_base_link():
    data = yaml.safe_load((PACKAGE_ROOT / "config" / "nav2_localization.yaml").read_text())
    amcl = data.get("amcl", {}).get("ros__parameters", {})
    assert amcl.get("base_frame_id", "base_link") == "base_link"


def test_mapper_params_online_async_yaml_loads():
    data = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "mapper_params_online_async.yaml").read_text())
    assert "slam_toolbox" in data


def test_helper_modules_import_cleanly():
    import slam_lab.slam_monitor  # noqa: F401
    import slam_lab.initial_pose_setter  # noqa: F401
