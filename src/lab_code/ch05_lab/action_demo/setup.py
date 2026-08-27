from setuptools import find_packages, setup
pkg = 'action_demo'
setup(name=pkg, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/'+pkg]),
                ('share/'+pkg, ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Student', description='动作通信实验包',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'server = action_demo.dishes_server:main',
        'client = action_demo.dishes_client:main',
    ]})
