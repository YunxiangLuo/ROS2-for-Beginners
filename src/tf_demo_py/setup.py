from setuptools import setup

package_name = "tf_demo_py"

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
    description="ROS 2 Python tf2 demo nodes.",
    license="BSD",
    entry_points={
        "console_scripts": [
            "coordinate_transformation = tf_demo_py.coordinate_transformation:main",
            "py_tf_broadcaster = tf_demo_py.py_tf_broadcaster:main",
            "py_tf_listener = tf_demo_py.py_tf_listener:main",
        ],
    },
)
