import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_vision'

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
        # config files -> install/robot_vision/share/robot_vision/config/
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',                    # TODO: your name
    maintainer_email='developer@todo.todo',    # TODO: your email
    description='Vision package: camera input, AprilTag detection, QR detection (skeleton).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # executable name = module:function
            'camera_node = robot_vision.camera_node:main',
            'apriltag_node = robot_vision.apriltag_node:main',
            'qr_node = robot_vision.qr_node:main',
        ],
    },
)
