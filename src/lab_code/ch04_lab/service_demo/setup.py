from setuptools import find_packages, setup
package_name = 'service_demo'
setup(
    name=package_name, version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Student', maintainer_email='student@example.com',
    description='服务通信实验包', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = service_demo.server:main',
            'client = service_demo.client:main',
        ],
    },
)
