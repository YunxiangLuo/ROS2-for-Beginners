from glob import glob
from pathlib import Path

from setuptools import setup


package_name = "robot_sim_demo"


def package_data_files():
    nested_package = "wheeltec_robot_urdf"
    data_files = [
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
        (
            "share/" + package_name + "/config",
            glob("config/*"),
        ),
        (
            "share/" + package_name + "/gui",
            glob("gui/*"),
        ),
        (
            "share/" + package_name + "/rviz",
            glob("rviz/*"),
        ),
        (
            "share/" + package_name + "/urdf",
            glob("urdf/*"),
        ),
        (
            "share/" + package_name + "/worlds",
            glob("worlds/*"),
        ),
    ]

    for path in glob("models/**/*", recursive=True):
        if Path(path).is_file():
            data_files.append(
                (
                    "share/" + package_name + "/" + Path(path).parent.as_posix(),
                    [path],
                )
            )
    for path in glob("wheeltec_robot_urdf/**/*", recursive=True):
        if Path(path).is_file():
            data_files.append(
                (
                    "share/" + package_name + "/" + Path(path).parent.as_posix(),
                    [path],
                )
            )
            relative_parent = Path(path).parent.relative_to(nested_package)
            data_files.append(
                (
                    "share/" + nested_package + "/" + relative_parent.as_posix(),
                    [path],
                )
            )
    data_files.append(
        (
            "share/ament_index/resource_index/packages",
            [nested_package + "/resource/" + nested_package],
        )
    )
    return data_files


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=package_data_files(),
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="tiger",
    maintainer_email="dev@example.com",
    description="ISCAS Museum Gazebo Sim demo with the safety inspection robot model.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera_info_publisher = robot_sim_demo.camera_info_publisher:main",
            "patrol_driver = robot_sim_demo.patrol_driver:main",
        ],
    },
)
