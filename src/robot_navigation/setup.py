import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.1.0',
    # robot_navigation/__init__.py + launch_common.py (helpers imported by the launch files).
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament resource index marker (required so ROS2 can find this package)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch files -> install/robot_navigation/share/robot_navigation/launch/
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # config files -> install/robot_navigation/share/robot_navigation/config/
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        # maps (README, map.yaml, map.pgm, *.posegraph, *.data) -> install/robot_navigation/share/robot_navigation/maps/
        (os.path.join('share', package_name, 'maps'),
            [f for f in glob(os.path.join('maps', '*')) if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',                    # TODO: your name
    maintainer_email='developer@todo.todo',    # TODO: your email
    description='Navigation package: EKF + slam_toolbox mapping, AMCL localization and Nav2 navigation (launch + config).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # robot_navigation has no nodes (launch + config only); standard packages are wired in launch/
        ],
    },
)
