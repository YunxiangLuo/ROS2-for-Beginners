"""Tests for vision_pickup_lab modules and delegation."""
import xml.etree.ElementTree as ElementTree
from pathlib import Path

import vision_pickup_lab.tf2_camera_broadcaster as broadcaster

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_pipeline_delegates_to_course_lab_utils():
    text = (PACKAGE_ROOT / "vision_pickup_lab" / "vision_pickup_pipeline.py").read_text(encoding='utf-8')
    assert "from course_lab_utils.vision_pickup_pipeline import main" in text


def test_pick_server_delegates_to_course_lab_utils():
    text = (PACKAGE_ROOT / "vision_pickup_lab" / "aruco_pick_server.py").read_text(encoding='utf-8')
    assert "from course_lab_utils.aruco_pick_server import main" in text


def test_tf2_camera_broadcaster_imports_and_converts_euler():
    q = broadcaster.TF2CameraBroadcaster.euler_to_quaternion(0.0, 0.0, 0.0)
    assert q[3] == 1.0  # identity quaternion w=1
    norm = sum(component * component for component in q) ** 0.5
    assert abs(norm - 1.0) < 1e-9


def test_setup_registers_three_console_scripts():
    text = (PACKAGE_ROOT / "setup.py").read_text(encoding='utf-8')
    for script in ("tf2_camera_broadcaster", "aruco_pick_server",
                   "vision_pickup_pipeline"):
        assert f"{script} = vision_pickup_lab.{script}:main" in text


def test_package_xml_declares_dependencies():
    root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
    deps = {dep.text for dep in root.findall("exec_depend")}
    assert "course_lab_utils" in deps
    assert "xarm_ros2_arm_only" in deps
    assert "tf2_ros" in deps
