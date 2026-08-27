from setuptools import setup

package_name = "param_demo_py"

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
    description="ROS 2 Python parameter demo.",
    license="BSD",
    entry_points={"console_scripts": ["param_demo = param_demo_py.param_demo:main"]},
)
