import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'robot_vision'


setup(
    name=package_name,
    version='0.1.0',

    # robot_vision/__init__.py 파일을 유지해야 한다.
    packages=find_packages(exclude=['test']),

    data_files=[
        # ROS2 패키지 검색용 resource marker
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),

        # 패키지 메타데이터
        (
            'share/' + package_name,
            ['package.xml'],
        ),

        # config/*.yaml -> share/robot_vision/config/
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')),
        ),
    ],

    # ROS 의존성은 package.xml에서 관리한다.
    # 이 항목만으로 torch/ultralytics가 설치되지는 않는다.
    install_requires=['setuptools'],
    zip_safe=True,

    # 기존 메타데이터 유지
    maintainer='developer',
    maintainer_email='developer@todo.todo',
    description=(
        'Vision package: camera input, AprilTag detection and pose, '
        'QR detection, YOLO object detection and person pose.'
    ),
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            # ros2 run robot_vision camera_node
            'camera_node = robot_vision.camera_node:main',

            # ros2 run robot_vision apriltag_node
            'apriltag_node = robot_vision.apriltag_node:main',

            # ros2 run robot_vision qr_node
            'qr_node = robot_vision.qr_node:main',

            # ros2 run robot_vision detector_node
            'detector_node = robot_vision.detector_node:main',

            # ros2 run robot_vision pose_node
            'pose_node = robot_vision.pose_node:main',
        ],
    },
)