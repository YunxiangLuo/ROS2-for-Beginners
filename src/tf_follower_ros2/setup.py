from setuptools import setup

package_name = "tf_follower_ros2"

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
                "launch/tf_follower.launch.py",
                "launch/tf_follower_demo.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS Academy",
    maintainer_email="anchuanxu@todo.todo",
    description="ROS 2 tf follower controller migrated from the ROS 1 tf_follower demo.",
    license="BSD",
    entry_points={
        "console_scripts": [
            "tf_follower = tf_follower_ros2.tf_follower:main",
            "fake_target_broadcaster = tf_follower_ros2.fake_target_broadcaster:main",
        ],
    },
)
