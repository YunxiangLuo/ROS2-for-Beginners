from setuptools import find_packages, setup

package_name = 'av_sensor_kit'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/default_sensors.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='CARLA传感器套件配置与管理',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_manager = av_sensor_kit.sensor_manager:main',
            'sensor_config = av_sensor_kit.sensor_config:main',
        ],
    },
)
