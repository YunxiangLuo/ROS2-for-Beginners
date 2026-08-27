from setuptools import find_packages, setup
import os
from glob import glob

pkg = 'nav_bringup_lab'
setup(
    name=pkg,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
        ('share/' + pkg, ['package.xml']),
        (os.path.join('share', pkg, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Student',
    description='Nav2 导航启动实验包',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'nav_goal_runner = nav_bringup_lab.nav_goal_runner:main',
    ]},
)
