"""Tests for moveit_pick_place_lab delegation to course_lab_utils."""
import xml.etree.ElementTree as ElementTree
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]

DELEGATIONS = {
    "pick_place_demo": "moveit_pick_place_demo",
    "obstacles_demo": "moveit_obstacles_demo",
    "beeline_demo": "moveit_beeline_demo",
    "attach_object_demo": "moveit_attach_demo",
    "target_publisher": "target_publisher",
}


def test_all_modules_delegate_to_course_lab_utils():
    for module, helper in DELEGATIONS.items():
        text = (PACKAGE_ROOT / "moveit_pick_place_lab" / f"{module}.py").read_text(encoding='utf-8')
        assert f"from course_lab_utils.{helper} import main" in text, (
            f"{module} does not delegate to {helper}")


def test_setup_registers_all_console_scripts():
    text = (PACKAGE_ROOT / "setup.py").read_text(encoding='utf-8')
    for module in DELEGATIONS:
        assert f"{module} = moveit_pick_place_lab.{module}:main" in text


def test_package_xml_declares_dependencies():
    root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
    deps = {dep.text for dep in root.findall("exec_depend")}
    assert "course_lab_utils" in deps
    assert "xarm_ros2_arm_only" in deps
