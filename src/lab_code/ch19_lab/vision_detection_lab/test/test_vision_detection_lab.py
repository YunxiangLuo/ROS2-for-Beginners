"""Tests for vision_detection_lab modules.

颜色/AR 检测委托 course_lab_utils；usb_cam 与 cv_bridge 演示为独立实现。
所有模块仅依赖 rclpy/sensor_msgs/cv_bridge，可安全导入。
"""
import xml.etree.ElementTree as ElementTree
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_detection_nodes_delegate_to_course_lab_utils():
    for module, helper in (
        ("color_detection_node", "color_detection_node"),
        ("ar_tag_detection_node", "aruco_detection_node"),
    ):
        text = (PACKAGE_ROOT / "vision_detection_lab" / f"{module}.py").read_text(encoding='utf-8')
        assert f"from course_lab_utils.{helper} import main" in text


def test_standalone_programs_import_cleanly():
    import vision_detection_lab.usb_cam_node  # noqa: F401
    import vision_detection_lab.cv_bridge_demo  # noqa: F401


def test_setup_registers_four_console_scripts():
    text = (PACKAGE_ROOT / "setup.py").read_text(encoding='utf-8')
    for script in ("usb_cam_node", "cv_bridge_demo",
                   "color_detection_node", "ar_tag_detection_node"):
        assert f"{script} = vision_detection_lab.{script}:main" in text


def test_package_xml_declares_cv_bridge():
    root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
    deps = {dep.text for dep in root.findall("exec_depend")}
    assert "cv_bridge" in deps
    assert "course_lab_utils" in deps
