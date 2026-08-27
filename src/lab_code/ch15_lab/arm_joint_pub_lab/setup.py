from setuptools import find_packages, setup

pkg = 'arm_joint_pub_lab'
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
    description='机械臂关节状态发布实验包（xArm）',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'hello_arm_node = arm_joint_pub_lab.hello_arm_node:main',
        'arm_joints_pub1 = arm_joint_pub_lab.arm_joints_pub1:main',
        'arm_gripper = arm_joint_pub_lab.arm_gripper:main',
        'gripper_open_close = arm_joint_pub_lab.gripper_open_close:main',
    ]},
)
