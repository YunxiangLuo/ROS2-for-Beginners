import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'navigation_lab'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='Chapter 11 Nav2 navigation laboratory exercises',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'go_to_pose_demo = navigation_lab.go_to_pose_demo:main',
            'follow_waypoints_demo = navigation_lab.follow_waypoints_demo:main',
            'recovery_demo = navigation_lab.recovery_demo:main',
            'scan_injector = navigation_lab.scan_injector:main',
            'nav_monitor = navigation_lab.nav_monitor:main',
            'waypoint_patrol = navigation_lab.waypoint_patrol:main',
        ],
    },
)
