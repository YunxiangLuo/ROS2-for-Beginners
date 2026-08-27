from setuptools import find_packages, setup
import os; from glob import glob
pkg = 'tf_demo'
setup(name=pkg, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/'+pkg]),
        ('share/'+pkg, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Student', description='TF2 坐标变换实验包',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'tf_broadcaster = tf_demo.tf_broadcaster:main',
        'tf_listener = tf_demo.tf_listener:main',
    ]})
