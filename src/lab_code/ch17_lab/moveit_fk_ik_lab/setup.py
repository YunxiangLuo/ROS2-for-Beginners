from setuptools import find_packages, setup

pkg = 'moveit_fk_ik_lab'
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
    description='MoveIt FK/IK 实验包（xArm）',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'fk_demo = moveit_fk_ik_lab.fk_demo:main',
        'ik_demo = moveit_fk_ik_lab.ik_demo:main',
        'fk_ik_exercise = moveit_fk_ik_lab.fk_ik_exercise:main',
        'rectangle_exercise = moveit_fk_ik_lab.rectangle_exercise:main',
    ]},
)
