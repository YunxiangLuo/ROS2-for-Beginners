from setuptools import find_packages, setup

pkg = 'vision_pickup_lab'
setup(
    name=pkg,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
        ('share/' + pkg, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Student',
    description='视觉引导抓取实验包（AR 码 + xArm）',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'tf2_camera_broadcaster = vision_pickup_lab.tf2_camera_broadcaster:main',
        'aruco_pick_server = vision_pickup_lab.aruco_pick_server:main',
        'vision_pickup_pipeline = vision_pickup_lab.vision_pickup_pipeline:main',
    ]},
)
