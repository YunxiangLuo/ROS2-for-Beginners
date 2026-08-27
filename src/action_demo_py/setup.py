from setuptools import setup

package_name = "action_demo_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS Academy",
    maintainer_email="anchuanxu@todo.todo",
    description="ROS 2 Python action_demo nodes.",
    license="BSD",
    entry_points={
        "console_scripts": [
            "dishes_server = action_demo_py.dishes_server:main",
            "dishes_client = action_demo_py.dishes_client:main",
        ],
    },
)
