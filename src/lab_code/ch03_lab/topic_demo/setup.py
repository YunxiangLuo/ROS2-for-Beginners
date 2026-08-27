from setuptools import find_packages, setup

package_name = 'topic_demo'

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
    description='ROS 2 话题通信实验包',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_pub = topic_demo.publisher:main',
            'gps_sub = topic_demo.subscriber:main',
            'qos_pub = topic_demo.qos_publisher:main',
            'square_driver = topic_demo.square_driver:main',
        ],
    },
)
