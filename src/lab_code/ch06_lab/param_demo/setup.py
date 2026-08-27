from setuptools import find_packages, setup
import os; from glob import glob
pkg = 'param_demo'
setup(name=pkg, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/'+pkg]),
        ('share/'+pkg, ['package.xml']),
        (os.path.join('share', pkg, 'launch'), glob('launch/*.py')),
        (os.path.join('share', pkg, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    tests_require=['pytest'],
    maintainer='Student', description='参数系统实验包',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'param_node = param_demo.param_demo:main',
        'speed_ctrl = param_demo.speed_controller:main',
    ]})
