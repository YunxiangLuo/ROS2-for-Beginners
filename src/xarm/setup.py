from glob import glob

from setuptools import setup


package_name = "xarm_ros2_arm_only"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (
            "share/" + package_name,
            ["package.xml", "README.md", "LICENSE"],
        ),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/urdf", glob("urdf/*.xacro")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config", glob("config/*.srdf")),
        ("share/" + package_name + "/config", glob("config/*.rviz")),
        ("share/" + package_name + "/config", glob("config/*.config")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Developer",
    maintainer_email="developer@example.invalid",
    description="Standalone xArm6 Gazebo Harmonic and MoveIt 2 simulation.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "arm_only_runtime_smoke = xarm_ros2_arm_only.runtime_smoke:main",
            "arm_only_moveit_sequence = xarm_ros2_arm_only.moveit_sequence:main",
        ],
    },
)
