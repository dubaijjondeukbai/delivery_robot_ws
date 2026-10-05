import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament resource index marker (required so ROS2 can find this package)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch files -> install/robot_bringup/share/robot_bringup/launch/
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # config files -> install/robot_bringup/share/robot_bringup/config/
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',                    # TODO: your name
    maintainer_email='developer@todo.todo',    # TODO: your email
    description='Master package: launches the whole delivery robot system and loads parameters.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # robot_bringup has no nodes (launch + config only)
        ],
    },
)
