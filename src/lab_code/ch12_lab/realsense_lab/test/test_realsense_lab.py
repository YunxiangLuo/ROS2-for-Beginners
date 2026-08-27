"""Tests for realsense_lab launch and bringup assets."""
import os
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CHAPTER_ROOT = PACKAGE_ROOT.parent


def test_launch_delegates_to_realsense2_camera():
    text = (PACKAGE_ROOT / "launch" / "realsense.launch.py").read_text(encoding='utf-8')
    assert "realsense2_camera" in text
    assert "rs_launch.py" in text
    assert "pointcloud" in text


def test_shell_bringup_script_exists():
    script = CHAPTER_ROOT / "realsense_launch.sh"
    assert script.is_file()
    text = script.read_text(encoding='utf-8')
    assert "rs_launch.py" in text


def test_package_xml_declares_realsense_dependency():
    import xml.etree.ElementTree as ElementTree
    root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
    deps = {dep.text for dep in root.findall("exec_depend")}
    assert "realsense2_camera" in deps
