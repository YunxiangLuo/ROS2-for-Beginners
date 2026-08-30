from setuptools import find_packages, setup

pkg = 'vision_llm_demo'
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
    description='视觉大模型服务实验包（离线 mock 与真实提供商可切换）',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'vision_llm_server = vision_llm_demo.vision_llm_server:main',
        'vision_llm_client = vision_llm_demo.vision_llm_client:main',
    ]},
)
