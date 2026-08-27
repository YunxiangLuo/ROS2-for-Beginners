from setuptools import setup

package_name = "urdf_demo_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/display_urdf_link_joint.launch.py",
                "launch/display_urdf_link_position.launch.py",
                "launch/display_urdf_color_geometry.launch.py",
                "launch/display_xacro.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/urdf",
            [
                "urdf/macros.xacro",
                "urdf/materials.xacro",
                "urdf/mybot.xacro",
            ],
        ),
        (
            "share/" + package_name + "/rviz",
            [
                "rviz/base_link.rviz",
                "rviz/mybot_link.rviz",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS Academy",
    maintainer_email="anchuanxu@todo.todo",
    description="ROS 2 URDF and xacro visualization demo.",
    license="BSD",
)
