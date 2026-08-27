"""Tests for slam_bringup_lab delegation."""
import xml.etree.ElementTree as ElementTree
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CHAPTER_ROOT = PACKAGE_ROOT.parent


def test_launch_uses_robot_sim_demo_harmonic_entry():
    text = (PACKAGE_ROOT / "launch" / "slam_bringup.launch.py").read_text(encoding="utf-8")
    assert "gazebo2.launch.py" in text
    assert "robot_sim_demo" in text
    # 旧脚本错误引用的包名不应出现
    assert "robot_sim_demo_ros2" not in text.replace(
        "slam_sim_demo_ros2", "")


def test_launch_uses_slam_sim_demo_entry():
    text = (PACKAGE_ROOT / "launch" / "slam_bringup.launch.py").read_text(encoding="utf-8")
    assert "slam_sim_demo_ros2" in text
    assert "slam_demo.launch.py" in text


def test_map_runner_delegates_to_slam_sim_demo():
    text = (PACKAGE_ROOT / "slam_bringup_lab" / "slam_map_runner.py").read_text(encoding="utf-8")
    assert "from slam_sim_demo_ros2.slam_map_runner import main" in text


def test_shell_bringup_references_were_fixed():
    """The chapter shell script uses the current Gazebo Sim entry point."""
    script = CHAPTER_ROOT / "slam_bringup.sh"
    if script.is_file():
        text = script.read_text(encoding="utf-8")
        assert "robot_sim_demo" in text
        assert "gazebo2.launch.py" in text
        assert (CHAPTER_ROOT / "slam_bringup_lab" / "README.md").is_file()


def test_package_xml_declares_sim_dependencies():
    root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
    deps = {dep.text for dep in root.findall("exec_depend")}
    assert "robot_sim_demo" in deps
    assert "slam_sim_demo_ros2" in deps
