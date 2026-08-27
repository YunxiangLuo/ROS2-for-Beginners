from setuptools import setup

package_name = "slam_sim_demo_ros2"

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
                "launch/slam_demo.launch.py",
                "launch/slam_depth_demo.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/params",
            [
                "params/slam_toolbox_params.yaml",
            ],
        ),
        (
            "share/" + package_name + "/rviz",
            [
                "rviz/slam.rviz",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS Academy",
    maintainer_email="anchuanxu@todo.todo",
    description="Minimal slam_toolbox migration of the ROS 1 slam_sim_demo package.",
    license="BSD",
    entry_points={
        "console_scripts": [
            "slam_map_runner = slam_sim_demo_ros2.slam_map_runner:main",
            "slam_save_reload_runner = slam_sim_demo_ros2.slam_save_reload_runner:main",
        ],
    },
)
