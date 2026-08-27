"""Source-level tests for the xarm_ros2_arm_only package assets and configs."""
import os
import xml.etree.ElementTree as ElementTree
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


REQUIRED_FILES = [
    "package.xml",
    "setup.py",
    "setup.cfg",
    "resource/xarm_ros2_arm_only",
    "README.md",
    "LICENSE",
    "launch/arm_only.launch.py",
    "launch/arm_only_move_group.launch.py",
    "urdf/arm_only_xarm.urdf.xacro",
    "worlds/arm_only.sdf",
    "config/arm_only_controllers.yaml",
    "config/arm_only_joint_limits.yaml",
    "config/arm_only_kinematics.yaml",
    "config/arm_only_ompl_planning.yaml",
    "config/moveit_controllers.yaml",
    "config/initial_positions.yaml",
    "config/xarm.srdf",
    "config/arm_only_moveit.rviz",
    "config/arm_only_gazebo.config",
    "xarm_ros2_arm_only/runtime_smoke.py",
    "xarm_ros2_arm_only/__init__.py",
]


def test_required_files_exist():
    for relative in REQUIRED_FILES:
        assert (PACKAGE_ROOT / relative).is_file(), f"missing {relative}"


def test_package_xml_declares_xarm_description_dependency():
    root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
    deps = {dep.text for dep in root.findall("exec_depend")}
    assert "xarm_description" in deps
    assert "joint_state_publisher" in deps
    assert "trac_ik_kinematics_plugin" in deps
    assert "moveit_ros_move_group" in deps


def test_controllers_yaml_targets_repository_joints():
    data = yaml.safe_load((PACKAGE_ROOT / "config/arm_only_controllers.yaml").read_text())
    xarm_joints = data["xarm_controller"]["ros__parameters"]["joints"]
    assert xarm_joints == [f"arm_{i}_joint" for i in range(1, 7)]
    gripper = data["gripper_controller"]["ros__parameters"]["joints"]
    assert gripper == ["gripper_1_joint"]
    declared = data["controller_manager"]["ros__parameters"]
    assert declared["joint_state_broadcaster"]["type"] == "joint_state_broadcaster/JointStateBroadcaster"
    assert declared["xarm_controller"]["type"] == "joint_trajectory_controller/JointTrajectoryController"
    assert declared["gripper_controller"]["type"] == "joint_trajectory_controller/JointTrajectoryController"


def test_srdf_defines_xarm_chain_and_gripper_group():
    root = ElementTree.parse(PACKAGE_ROOT / "config/xarm.srdf").getroot()
    groups = {g.get("name"): g for g in root.findall("group")}
    assert "xarm" in groups
    chain = groups["xarm"].find("chain")
    assert chain is not None
    assert chain.get("base_link") == "base_link"
    assert chain.get("tip_link") == "gripper_centor_link"
    assert "gripper" in groups


def test_initial_positions_cover_all_arm_joints():
    data = yaml.safe_load((PACKAGE_ROOT / "config/initial_positions.yaml").read_text())
    positions = data["initial_positions"]
    for joint in [f"arm_{i}_joint" for i in range(1, 7)]:
        assert joint in positions
    assert "gripper_1_joint" in positions


def test_world_sdf_is_valid_and_has_no_models():
    root = ElementTree.parse(PACKAGE_ROOT / "worlds/arm_only.sdf").getroot()
    world = root.find("world")
    assert world is not None
    assert world.get("name") == "arm_only"
    # The arm-only world intentionally has no model/include elements.
    assert world.find("model") is None
    assert world.find("include") is None
    assert world.find("plugin[@filename='gz-sim-physics-system']") is not None


def test_launch_provides_use_gazebo_and_run_move_group_args():
    text = (PACKAGE_ROOT / "launch/arm_only.launch.py").read_text()
    assert 'DeclareLaunchArgument("use_gazebo"' in text
    assert 'DeclareLaunchArgument("run_move_group"' in text
    assert 'DeclareLaunchArgument("use_rviz"' in text
    # Mock joint state publisher must run when Gazebo is disabled.
    assert "joint_state_publisher" in text
    assert "UnlessCondition" in text


def test_runtime_smoke_targets_repository_joints_and_controllers():
    text = (PACKAGE_ROOT / "xarm_ros2_arm_only/runtime_smoke.py").read_text()
    assert "ARM_JOINTS" in text
    assert "range(1, 7)" in text
    for controller in ("joint_state_broadcaster", "xarm_controller", "gripper_controller"):
        assert controller in text
    assert "/plan_kinematic_path" in text
    assert "/execute_trajectory" in text


def test_setup_installs_all_assets():
    text = (PACKAGE_ROOT / "setup.py").read_text()
    for pattern in ("launch/*.launch.py", "urdf/*.xacro", "worlds/*.sdf",
                    "config/*.yaml", "config/*.srdf", "config/*.rviz", "config/*.config"):
        assert pattern in text, f"setup.py does not install {pattern}"
    assert "tests_require" in text
