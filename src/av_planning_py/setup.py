from setuptools import find_packages, setup

package_name = 'av_planning_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/planner_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tiger',
    maintainer_email='tiger@example.com',
    description='全局/局部路径规划节点',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_planner = av_planning_py.global_planner:main',
            'waypoint_generator = av_planning_py.waypoint_generator:main',
            'planning_server = av_planning_py.planning_server:main',
        ],
    },
)
