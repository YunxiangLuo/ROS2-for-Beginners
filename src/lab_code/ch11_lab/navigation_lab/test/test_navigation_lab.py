"""Tests for navigation_lab assets and AMCL frame consistency."""
import xml.etree.ElementTree as ElementTree
from pathlib import Path

import pytest
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_navigation_lab_yaml_uses_base_link_for_amcl():
    data = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "navigation_lab.yaml").read_text())
    amcl = data["amcl"]["ros__parameters"]
    assert amcl["base_frame_id"] == "base_link"
    assert amcl["odom_frame_id"] == "odom"
    assert amcl["global_frame_id"] == "map"
    assert amcl["scan_topic"] == "/scan"


def test_costmaps_use_base_link_robot_base_frame():
    data = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "navigation_lab.yaml").read_text())
    assert data["local_costmap"]["local_costmap"]["ros__parameters"]["robot_base_frame"] == "base_link"
    assert data["global_costmap"]["global_costmap"]["ros__parameters"]["robot_base_frame"] == "base_link"


def test_helper_modules_import_cleanly():
    import navigation_lab.nav_utils  # noqa: F401


def test_scan_injector_uses_laser_link_frame():
    text = (PACKAGE_ROOT / "navigation_lab" / "scan_injector.py").read_text()
    assert "laser_link" in text or "laser" in text


def test_nav_bringup_launch_exists():
    assert (PACKAGE_ROOT / "launch" / "nav_bringup.launch.py").is_file()


def test_package_xml_declares_nav2_commander():
    root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
    deps = {dep.text for dep in root.findall("exec_depend")}
    assert "nav2_simple_commander" in deps
    assert "nav2_bringup" in deps
