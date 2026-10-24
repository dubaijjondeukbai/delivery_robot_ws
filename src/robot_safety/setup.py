import os

from setuptools import find_packages, setup


package_name = 'robot_safety'


setup(
    name=package_name,
    version='0.0.0',

    # 실제 Python 패키지를 설치한다.
    packages=find_packages(
        exclude=['test', 'test.*'],
    ),

    data_files=[
        # ROS 2 패키지 검색을 위한 등록 파일.
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),

        # ROS 2 패키지 정보.
        (
            os.path.join('share', package_name),
            ['package.xml'],
        ),

        # 안전 설정 파일.
        # 설치 위치:
        # <package_share>/config/safety.yaml
        #
        # 파일을 만들지 않았다면 빌드 시 오류가 발생한다.
        (
            os.path.join('share', package_name, 'config'),
            ['config/safety.yaml'],
        ),
    ],

    # 기존 타입 정보 표시 파일을 유지한다.
    package_data={
        package_name: ['py.typed'],
    },

    # ROS 의존성은 package.xml에서 관리한다.
    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='seojs',
    maintainer_email='seojs@todo.todo',

    description=(
        'Delivery robot safety: velocity command gating, '
        'E-stop monitoring and LiDAR obstacle detection.'
    ),

    # 기존 선언 유지. 배포 라이선스는 별도로 확정해야 한다.
    license='TODO: License declaration',

    extras_require={
        'test': ['pytest'],
    },

    entry_points={
        'console_scripts': [
            (
                'safety_manager_node = '
                'robot_safety.safety_manager_node:main'
            ),
            (
                'lidar_obstacle_node = '
                'robot_safety.lidar_obstacle_node:main'
            ),
        ],
    },
)