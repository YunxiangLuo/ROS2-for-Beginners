from setuptools import find_packages, setup

package_name = 'hello_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='ROS 2 Python 节点编程实验包',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_node = hello_pkg.hello_node:main',
            'logger_node = hello_pkg.logger_demo:main',
            'odom_monitor = hello_pkg.odom_monitor:main',
        ],
    },
)
