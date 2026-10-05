# Python 패키지 설치를 위한 setuptools 함수 import
from setuptools import find_packages, setup

# 패키지 이름
package_name = 'robot_safety'

# 패키지 설치 설정
setup(
    # ROS2 패키지 이름
    name=package_name,

    # 버전
    version='0.0.0',

    # robot_safety Python package를 자동으로 찾음
    # test 폴더는 제외
    packages=find_packages(exclude=['test']),

    # ROS2가 패키지를 찾기 위해 필요한 파일 설치
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],

    # py.typed 파일 포함
    package_data={
        '': ['py.typed']
    },

    # Python 설치 dependency
    install_requires=['setuptools'],

    # 압축 설치 가능 여부
    zip_safe=True,

    # 관리자 정보
    maintainer='seojs',
    maintainer_email='seojs@todo.todo',

    # 패키지 설명
    description='Delivery robot safety package',

    # 추후 MIT 등으로 변경 가능
    license='TODO: License declaration',

    # 테스트 dependency
    extras_require={
        'test': [
            'pytest',
        ],
    },

    # ROS2에서 실행 가능한 Python node 등록
    entry_points={
        'console_scripts': [

            # ros2 run robot_safety safety_manager_node
            'safety_manager_node = robot_safety.safety_manager_node:main',

            # ros2 run robot_safety lidar_obstacle_node
            'lidar_obstacle_node = robot_safety.lidar_obstacle_node:main',
        ],
    },
)