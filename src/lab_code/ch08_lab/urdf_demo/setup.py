from setuptools import find_packages, setup
import os; from glob import glob
pkg = 'urdf_demo'
setup(name=pkg, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/'+pkg]),
        ('share/'+pkg, ['package.xml']),
        (os.path.join('share', pkg, 'launch'), glob('launch/*.py')),
        (os.path.join('share', pkg, 'urdf'), glob('urdf/*.xacro') + glob('urdf/*.urdf')),
        (os.path.join('share', pkg, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    tests_require=['pytest'],
    maintainer='Student', description='URDF/Xacro 机器人建模实验包',
    license='Apache-2.0',
    entry_points={'console_scripts': []})
