import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_sim'

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
        # launch files -> install/robot_sim/share/robot_sim/launch/
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # bridge config -> install/robot_sim/share/robot_sim/config/
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        # worlds + textures -> install/robot_sim/share/robot_sim/worlds/...
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'worlds', 'materials', 'textures'),
            glob(os.path.join('worlds', 'materials', 'textures', '*.png'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',                    # TODO: your name
    maintainer_email='developer@todo.todo',    # TODO: your email
    description='Gazebo simulation (world, spawn, ros_gz_bridge) of the delivery robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # robot_sim has no nodes (world + launch + bridge config only)
        ],
    },
)
