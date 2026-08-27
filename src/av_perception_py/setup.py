from setuptools import setup
from setuptools import find_packages

package_name = 'av_perception_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/perception_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='自动驾驶感知节点(目标检测/跟踪)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = av_perception_py.object_detector:main',
            'lidar_detector = av_perception_py.lidar_detector:main',
            'fusion_node = av_perception_py.fusion_node:main',
        ],
    },
)
