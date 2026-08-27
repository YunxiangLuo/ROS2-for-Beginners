"""Tests for nav_bringup_lab delegation.

注意：nav_goal_runner 传递导入 nav2_simple_commander，在无 ROS 图环境下
导入可能阻塞，因此本测试通过文本校验委托关系而非直接导入。
"""
import xml.etree.ElementTree as ElementTree
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_launch_uses_robot_sim_demo_harmonic_entry():
    text = (PACKAGE_ROOT / "launch" / "nav_bringup.launch.py").read_text(encoding="utf-8")
    assert "gazebo2.launch.py" in text
    assert "robot_sim_demo" in text


def test_launch_uses_navigation_sim_demo_entry():
    text = (PACKAGE_ROOT / "launch" / "nav_bringup.launch.py").read_text(encoding="utf-8")
    assert "navigation_sim_demo_ros2" in text
    assert "nav2_demo.launch.py" in text


def test_goal_runner_delegates_to_navigation_sim_demo():
    text = (PACKAGE_ROOT / "nav_bringup_lab" / "nav_goal_runner.py").read_text(encoding="utf-8")
    assert "from navigation_sim_demo_ros2.nav_goal_runner import main" in text


def test_package_xml_declares_sim_dependencies():
    root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
    deps = {dep.text for dep in root.findall("exec_depend")}
    assert "robot_sim_demo" in deps
    assert "navigation_sim_demo_ros2" in deps
