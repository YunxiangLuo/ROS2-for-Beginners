"""Tests for sim_demo launch delegation to robot_sim_demo."""
import os
import xml.etree.ElementTree as ElementTree
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_launch_delegates_to_robot_sim_demo_harmonic_entry():
    text = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text(encoding="utf-8")
    assert "gazebo2.launch.py" in text
    assert "robot_sim_demo" in text
    assert "gazebo_ros" not in text
    assert "spawn_entity" not in text
    assert "empty.world" not in text


def test_launch_exposes_gui_rviz_drive_arguments():
    text = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text(encoding="utf-8")
    for arg in ("gui", "rviz", "drive"):
        assert f"DeclareLaunchArgument('{arg}'" in text, f"missing {arg} argument"


def test_package_xml_depends_on_robot_sim_demo_not_gazebo_ros():
    root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
    deps = {dep.text for dep in root.findall("exec_depend")}
    assert "robot_sim_demo" in deps
    assert "gazebo_ros" not in deps


def test_setup_does_not_install_missing_worlds_or_config():
    text = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    assert "worlds/*.world" not in text
    assert "config/*.yaml" not in text
