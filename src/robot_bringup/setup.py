from glob import glob
import os

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
    # colcon test 가 pytest 로 test/ 를 실행하도록 한다(robot_safety 와 같은 방식).
    extras_require={
        'test': ['pytest'],
    },
    maintainer='developer',                    # TODO: your name
    maintainer_email='developer@todo.todo',    # TODO: your email
    description='Master package: launches the whole delivery robot system and loads parameters.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # 관찰 전용 진단 노드(구독만 함): Nav2 -> Safety -> 구동 -> EKF 명령 흐름 기록
            'motion_chain_monitor = robot_bringup.motion_chain_monitor:main',
            # 전시·시연용 지점 저장/이동/순회 (Nav2 /navigate_to_pose)
            'stations = robot_bringup.stations:main',
            # QR 배달 시연: 출발 QR -> Nav2 -> 정지 확인 -> 도착 QR -> ESP32 뚜껑
            'delivery_demo = robot_bringup.delivery_demo:main',
            # 가짜 ESP32 뚜껑 (펌웨어 없이 delivery_demo 시험)
            'mock_lid_esp32 = robot_bringup.mock_lid_esp32:main',
            # Pi 모니터에 미션 상태 표시 (pygame)
            'delivery_display = robot_bringup.delivery_display:main',
            # 운영자 콘솔: 키 입력으로 START/OPEN/CLOSE/CANCEL/RESET
            'delivery_console = robot_bringup.delivery_console:main',
        ],
    },
)
