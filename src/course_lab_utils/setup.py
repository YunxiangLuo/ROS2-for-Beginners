from setuptools import find_packages, setup


package_name = "course_lab_utils"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS2 Course",
    maintainer_email="course@example.com",
    description="Shared ROS 2 Jazzy helpers for executable course labs.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
