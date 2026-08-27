from setuptools import setup

package_name = "navigation_sim_demo_ros2"

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
                "launch/nav2_demo.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/maps",
            [
                "maps/Software_Museum.yaml",
                "maps/Software_Museum.pgm",
            ],
        ),
        (
            "share/" + package_name + "/params",
            [
                "params/nav2_params.yaml",
            ],
        ),
        (
            "share/" + package_name + "/rviz",
            [
                "rviz/navigation.rviz",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS Academy",
    maintainer_email="anchuanxu@todo.todo",
    description="Minimal Nav2 migration of the ROS 1 navigation_sim_demo package.",
    license="BSD",
    entry_points={
        "console_scripts": [
            "initial_pose_publisher = navigation_sim_demo_ros2.initial_pose_publisher:main",
            "nav_goal_runner = navigation_sim_demo_ros2.nav_goal_runner:main",
            "nav2_lifecycle_runner = navigation_sim_demo_ros2.nav2_lifecycle_runner:main",
        ],
    },
)
