from setuptools import setup

package_name = 'base_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='统一底盘节点，集成全部 SDK 指令',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'unified_chassis = base_node.chassis_node:main',
        ],
    },
)