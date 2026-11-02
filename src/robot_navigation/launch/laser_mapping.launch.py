"""
[Navigation] laser_mapping.launch.py — 바퀴 odom 없이 LiDAR 만으로 지도 만들기.

    # LiDAR 를 노트북에 꽂고 들고(또는 카트로) 다니며
    ros2 launch robot_navigation laser_mapping.launch.py rviz:=true
    # 로봇에 단 LiDAR 로, 바퀴 odom 없이 로봇을 손으로 밀며 (URDF 의 LiDAR 위치 사용)
    ros2 launch robot_navigation laser_mapping.launch.py mount:=robot
    # 저장 (다 돈 뒤, 출발점으로 돌아와 한 바퀴를 닫고 나서)
    ros2 run nav2_map_server map_saver_cli -f ~/maps/roboworld \
        --ros-args -p map_subscribe_transient_local:=true

LiDAR 드라이버(sllidar_ros2, frame_id:=laser_link)는 따로 띄운다.

    LiDAR ──/scan──► slam_toolbox (async, 스캔 매칭만) ──/map + TF map->odom
    고정 TF odom -> base_link (움직임 0) ─┘
    mount: handheld = base_link -> laser_link 고정(같은 점), robot = robot_description 의 URDF,
           none = 다른 곳(robot.launch.py 등)이 base_link -> laser_link 를 낸다

odom 이 늘 0 이므로 slam_toolbox 가 0.2 s 마다 스캔을 맞춰 움직임을 찾는다(config/slam_laser_only.yaml,
시뮬레이션 결과도 거기). 0.3 m/s 이하로 아주 천천히, 회전은 초당 30° 이하, 모서리·회전 뒤 1~2 초 멈춘다.
LiDAR 는 로봇에 달린 높이와 같게, 수평으로. 사람이 많을 때보다 비었을 때가 좋다.
바퀴 odom 이 있으면 mapping.launch.py(EKF + slam_toolbox)가 더 정확하다.
EKF·robot.launch.py nav_mode:=mapping 과 함께 띄우지 않는다(odom -> base_link 를 두 곳에서 내면 안 됨).
mount:=robot 이면 robot.launch.py·base.launch.py 도 끈 상태로(로봇을 손으로 민다).
"""

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

from robot_navigation.launch_common import config_path

MOUNTS = ('handheld', 'robot', 'none')


def _truthy(text: str) -> bool:
    return text.strip().lower() in ('1', 'true', 'yes', 'on')


def _build(context):
    mount = LaunchConfiguration('mount').perform(context).strip().lower()
    if mount not in MOUNTS:
        raise RuntimeError(f'mount must be one of {MOUNTS}')
    use_sim_time = _truthy(LaunchConfiguration('use_sim_time').perform(context))
    scan_topic = LaunchConfiguration('scan_topic').perform(context)
    laser_frame = LaunchConfiguration('laser_frame').perform(context)

    actions = [
        LogInfo(msg=f'[laser_mapping] mount={mount} scan={scan_topic} laser_frame={laser_frame} '
                    'odom->base_link fixed (laser-only scan matching)'),
        # odom -> base_link: 움직임 0 (바퀴 odom 없음). 움직임은 slam_toolbox 의 map -> odom 에 들어간다.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='laser_only_odom_tf', output='log',
             arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link'],
             parameters=[{'use_sim_time': use_sim_time}]),
    ]
    if mount == 'handheld':
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='laser_only_laser_tf', output='log',
            arguments=['--frame-id', 'base_link', '--child-frame-id', laser_frame],
            parameters=[{'use_sim_time': use_sim_time}]))
    elif mount == 'robot':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('robot_description'), '/launch/description.launch.py']),
            launch_arguments={'sim': 'false', 'use_sim_time': str(use_sim_time).lower()}.items()))

    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            LaunchConfiguration('laser_only_params_file'),
            {'use_sim_time': use_sim_time, 'use_lifecycle_manager': False},
        ],
        remappings=[('scan', scan_topic), ('/scan', scan_topic)],
    )
    # mapping.launch.py 와 같은 방식: 뜨자마자 CONFIGURE, inactive 가 되면 ACTIVATE (lifecycle manager 없음)
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam_toolbox,
        start_state='configuring',
        goal_state='inactive',
        entities=[
            LogInfo(msg='[laser_mapping] slam_toolbox configured -> activating'),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_toolbox),
                transition_id=Transition.TRANSITION_ACTIVATE)),
        ],
    ))
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam_toolbox),
        transition_id=Transition.TRANSITION_CONFIGURE))
    actions += [slam_toolbox, activate, configure]

    if _truthy(LaunchConfiguration('rviz').perform(context)):
        rviz_config = LaunchConfiguration('rviz_config').perform(context).strip()
        if not rviz_config:
            try:   # nav2 기본 view (Fixed Frame: map). nav2_bringup 이 없으면 빈 RViz.
                rviz_config = os.path.join(get_package_share_directory('nav2_bringup'),
                                           'rviz', 'nav2_default_view.rviz')
            except PackageNotFoundError:
                rviz_config = ''
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', rviz_config] if os.path.isfile(rviz_config) else [],
            parameters=[{'use_sim_time': use_sim_time}]))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'mount', default_value='handheld',
            description='handheld | robot | none (who publishes base_link->laser)'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument(
            'laser_frame', default_value='laser_link',
            description='LaserScan frame_id (sllidar_ros2 frame_id:=laser_link)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config', default_value='',
            description="RViz config ('' = nav2_bringup default view, Fixed Frame: map)"),
        DeclareLaunchArgument('slam_params_file', default_value=config_path('slam.yaml')),
        DeclareLaunchArgument('laser_only_params_file',
                              default_value=config_path('slam_laser_only.yaml')),
        OpaqueFunction(function=_build),
    ])
