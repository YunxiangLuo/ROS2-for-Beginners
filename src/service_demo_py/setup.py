from setuptools import setup

package_name = "service_demo_py"

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
    description="ROS 2 Python service_demo nodes.",
    license="BSD",
    entry_points={
        "console_scripts": [
            "server_demo = service_demo_py.server_demo:main",
            "client_demo = service_demo_py.client_demo:main",
        ],
    },
)
