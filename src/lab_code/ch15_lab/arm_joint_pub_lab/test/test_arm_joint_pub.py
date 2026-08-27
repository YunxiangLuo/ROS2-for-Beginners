"""Tests for arm_joint_pub_lab modules and xArm joint naming."""
import xml.etree.ElementTree as ElementTree
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]

XARM_JOINTS = {
    "arm_1_joint", "arm_2_joint", "arm_3_joint",
    "arm_4_joint", "arm_5_joint", "arm_6_joint",
    "gripper_1_joint", "gripper_2_joint",
}


def test_all_modules_import_cleanly():
    import arm_joint_pub_lab.hello_arm_node  # noqa: F401
    import arm_joint_pub_lab.arm_joints_pub1  # noqa: F401
    import arm_joint_pub_lab.arm_gripper  # noqa: F401
    import arm_joint_pub_lab.gripper_open_close  # noqa: F401


def test_arm_joints_pub1_uses_xarm_joint_names():
    text = (PACKAGE_ROOT / "arm_joint_pub_lab" / "arm_joints_pub1.py").read_text(encoding='utf-8')
    for joint in XARM_JOINTS:
        assert joint in text, f"{joint} missing from arm_joints_pub1"


def test_gripper_modules_reference_xarm_gripper_joints():
    for module in ("arm_gripper.py", "gripper_open_close.py"):
        text = (PACKAGE_ROOT / "arm_joint_pub_lab" / module).read_text(encoding='utf-8')
        assert "gripper_1_joint" in text or "gripper" in text.lower(), (
            f"{module} does not reference gripper joints")


def test_setup_registers_four_console_scripts():
    text = (PACKAGE_ROOT / "setup.py").read_text(encoding='utf-8')
    for script in ("hello_arm_node", "arm_joints_pub1",
                   "arm_gripper", "gripper_open_close"):
        assert f"{script} = arm_joint_pub_lab.{script}:main" in text


def test_hello_arm_node_is_generic_teaching_example():
    """hello_arm_node 使用通用 joint1/finger 命名，是独立教学示例（非 xArm）."""
    text = (PACKAGE_ROOT / "arm_joint_pub_lab" / "hello_arm_node.py").read_text(encoding='utf-8')
    assert "joint_states" in text
