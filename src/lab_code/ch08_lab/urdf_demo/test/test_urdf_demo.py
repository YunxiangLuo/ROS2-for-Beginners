"""Tests for urdf_demo assets and launch references."""
import os
import xml.etree.ElementTree as ElementTree
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_xacro_file_exists_and_is_robot():
    xacro = PACKAGE_ROOT / "urdf" / "simple_robot.xacro"
    assert xacro.is_file()
    text = xacro.read_text(encoding='utf-8')
    assert "<robot" in text
    assert "base_link" in text


def test_display_rviz_config_exists_and_references_robot_description():
    rviz = PACKAGE_ROOT / "rviz" / "display.rviz"
    assert rviz.is_file(), "rviz/display.rviz is missing"
    text = rviz.read_text(encoding='utf-8')
    assert "/robot_description" in text
    assert "RobotModel" in text


def test_launch_uses_xacro_and_provides_rviz_config():
    text = (PACKAGE_ROOT / "launch" / "display.launch.py").read_text(encoding='utf-8')
    assert "simple_robot.xacro" in text
    assert "display.rviz" in text
    assert "robot_state_publisher" in text
    assert "joint_state_publisher_gui" in text


def test_setup_installs_rviz_and_urdf_assets():
    text = (PACKAGE_ROOT / "setup.py").read_text(encoding='utf-8')
    assert "urdf/*.xacro" in text
    assert "rviz/*.rviz" in text


def test_package_xml_declares_xacro_and_rviz_deps():
    root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
    deps = {dep.text for dep in root.findall("exec_depend")}
    assert "xacro" in deps
    assert "robot_state_publisher" in deps
    assert "rviz2" in deps
    assert "joint_state_publisher_gui" in deps
