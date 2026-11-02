from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robot_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    extras_require={
        'test': ['pytest'],
    },
    maintainer='seojs',
    maintainer_email='sjs0782@uos.ac.kr',
    description='4-wheel independent steering base bridge: /cmd_vel_safe <-> STM32 serial, '
                '/wheel/odom, /estop_state, and a fake STM32 for bench tests.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'base_bridge_node = robot_base.base_bridge_node:main',
            'mock_stm32 = robot_base.mock_stm32:main',
            'steer_sim_node = robot_base.steer_sim_node:main',
        ],
    },
)
