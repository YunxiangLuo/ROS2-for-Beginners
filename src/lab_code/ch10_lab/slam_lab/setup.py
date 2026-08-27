import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'slam_lab'


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
        (
            os.path.join('share', package_name, 'config', 'cartographer'),
            glob('config/cartographer/*.lua'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='SLAM, AMCL and Cartographer laboratory helpers',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'slam_monitor = slam_lab.slam_monitor:main',
            'set_initial_pose = slam_lab.initial_pose_setter:main',
            'amcl_evaluator = slam_lab.amcl_evaluator:main',
            'save_cartographer_state = slam_lab.cartographer_state_saver:main',
        ],
    },
)
