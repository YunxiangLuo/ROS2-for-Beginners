from setuptools import find_packages, setup

pkg = 'moveit_pick_place_lab'
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
    description='MoveIt 抓取放置实验包（xArm）',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'pick_place_demo = moveit_pick_place_lab.pick_place_demo:main',
        'obstacles_demo = moveit_pick_place_lab.obstacles_demo:main',
        'beeline_demo = moveit_pick_place_lab.beeline_demo:main',
        'attach_object_demo = moveit_pick_place_lab.attach_object_demo:main',
        'target_publisher = moveit_pick_place_lab.target_publisher:main',
    ]},
)
