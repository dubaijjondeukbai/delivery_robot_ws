import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_description'

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
        # launch files -> install/robot_description/share/robot_description/launch/
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # xacro / urdf -> install/robot_description/share/robot_description/urdf/
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.xacro')) + glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',                    # TODO: your name
    maintainer_email='developer@todo.todo',    # TODO: your email
    description='URDF/xacro model of the delivery robot and its robot_state_publisher launch.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # robot_description has no nodes (model + launch only)
        ],
    },
)
