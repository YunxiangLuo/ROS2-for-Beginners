"""Tests for moveit_fk_ik_lab programs.

程序传递导入 moveit.planning（MoveItPy），在无 ROS 图环境下导入缓慢，
因此通过文本校验关键结构，不直接导入演示模块。
"""
import xml.etree.ElementTree as ElementTree
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]

PROGRAMS = ("fk_demo.py", "ik_demo.py", "fk_ik_exercise.py", "rectangle_exercise.py")


def test_all_programs_exist():
    for program in PROGRAMS:
        assert (PACKAGE_ROOT / "moveit_fk_ik_lab" / program).is_file()


def test_programs_target_xarm_group():
    for program in PROGRAMS:
        text = (PACKAGE_ROOT / "moveit_fk_ik_lab" / program).read_text(encoding='utf-8')
        assert '"xarm"' in text, f"{program} does not target the xarm group"


def test_programs_use_course_lab_utils_helpers():
    for program in PROGRAMS:
        text = (PACKAGE_ROOT / "moveit_fk_ik_lab" / program).read_text(encoding='utf-8')
        assert "course_lab_utils.moveit2" in text, (
            f"{program} does not use course_lab_utils helpers")


def test_ik_demo_targets_gripper_centor_link():
    text = (PACKAGE_ROOT / "moveit_fk_ik_lab" / "ik_demo.py").read_text(encoding='utf-8')
    assert "gripper_centor_link" in text


def test_setup_registers_four_console_scripts():
    text = (PACKAGE_ROOT / "setup.py").read_text(encoding='utf-8')
    for script in ("fk_demo", "ik_demo", "fk_ik_exercise", "rectangle_exercise"):
        assert f"{script} = moveit_fk_ik_lab.{script}:main" in text


def test_package_xml_declares_xarm_dependency():
    root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
    deps = {dep.text for dep in root.findall("exec_depend")}
    assert "xarm_ros2_arm_only" in deps
    assert "course_lab_utils" in deps
