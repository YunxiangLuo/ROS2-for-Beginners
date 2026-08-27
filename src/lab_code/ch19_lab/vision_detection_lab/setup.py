from setuptools import find_packages, setup

pkg = 'vision_detection_lab'
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
    description='视觉检测实验包（相机/cv_bridge/颜色/AR码）',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'usb_cam_node = vision_detection_lab.usb_cam_node:main',
        'cv_bridge_demo = vision_detection_lab.cv_bridge_demo:main',
        'color_detection_node = vision_detection_lab.color_detection_node:main',
        'ar_tag_detection_node = vision_detection_lab.ar_tag_detection_node:main',
    ]},
)
