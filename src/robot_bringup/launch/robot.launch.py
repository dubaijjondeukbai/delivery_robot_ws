"""배달로봇 master launch. 기본값은 주행 비활성이다.

설정 순서: 패키지 YAML -> robot.yaml -> 명시적 launch 설정.
안전 노드 두 개는 항상 실행한다.
센서 드라이버/STM32 bridge/실물 E-stop 입력 발행자는 별도로 필요하다.

sim:=true 일 때:
  * sim_estop (기본 = sim): robot_sim 의 sim_estop_publisher 가 /estop_state=false 를
    대신 발행한다. sim:=false 에서 sim_estop:=true 는 거부한다(실물에서 가짜 E-stop 금지).
  * sim_amcl_init (기본 = sim): AMCL 초기 위치를 Gazebo 스폰 위치(sim_x/sim_y/sim_yaw)로
    자동 설정한다. 지도를 기본 스폰 위치에서 만들었다는 가정이며, 아니면
    sim_amcl_init:=false 로 끄고 RViz 2D Pose Estimate 를 쓴다.
  * rviz:=true: use_sim_time 을 맞춘 RViz(nav2 기본 view)를 함께 띄운다.
"""

import math
import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from robot_navigation.launch_common import (
    amcl_initial_pose_parameters,
    declare_common_arguments,
    load_wiring,
)


def _share(package, *parts):
    return os.path.join(
        get_package_share_directory(package),
        *parts,
    )


def _boolean(context, name):
    return ParameterValue(
        LaunchConfiguration(name),
        value_type=bool,
    ).evaluate(context)


def _include(package, filename, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _share(package, 'launch', filename)
        ),
        launch_arguments=arguments.items(),
    )


def _build_system(context, wiring):
    mode = LaunchConfiguration('nav_mode').perform(context)

    if mode not in ('none', 'mapping', 'localization', 'navigation'):
        raise RuntimeError(
            'nav_mode must be none/mapping/localization/navigation'
        )

    sim = _boolean(context, 'sim')
    use_sim_time = _boolean(context, 'use_sim_time')
    motion = _boolean(context, 'motion_enabled')
    second_camera = _boolean(context, 'enable_camera_2')
    sim_estop = _boolean(context, 'sim_estop')
    sim_amcl_init = _boolean(context, 'sim_amcl_init')
    rviz = _boolean(context, 'rviz')

    if sim and not use_sim_time:
        raise RuntimeError(
            'sim=true requires use_sim_time=true'
        )

    if sim and second_camera:
        raise RuntimeError(
            'The current Gazebo model provides only one camera'
        )

    # 가짜 E-stop 발행자는 시뮬레이션 밖에서 절대 실행하지 않는다.
    if sim_estop and not sim:
        raise RuntimeError(
            'sim_estop=true is allowed only with sim=true'
        )

    if sim_amcl_init and not sim:
        raise RuntimeError(
            'sim_amcl_init applies only to sim=true'
        )

    if _boolean(context, 'publish_sensor_tf'):
        raise RuntimeError(
            'Master launch uses URDF sensor TF; '
            'publish_sensor_tf must be false'
        )

    if motion and mode != 'navigation':
        raise RuntimeError(
            'motion_enabled applies only to nav_mode=navigation'
        )

    sim_command_topic = LaunchConfiguration(
        'sim_cmd_vel_topic'
    ).perform(context)

    if sim_command_topic != '/cmd_vel_safe':
        raise RuntimeError(
            'Master simulation must use '
            'sim_cmd_vel_topic=/cmd_vel_safe'
        )

    bond = ParameterValue(
        LaunchConfiguration('bond_timeout'),
        value_type=float,
    ).evaluate(context)

    if not math.isfinite(bond) or bond < 0:
        raise RuntimeError(
            'bond_timeout must be finite and >= 0'
        )

    for name, parameter in amcl_initial_pose_parameters().items():
        value = parameter.evaluate(context)

        if name.startswith('initial_pose.') and not math.isfinite(value):
            raise RuntimeError(f'{name} must be finite')

    robot_config = _share(
        'robot_bringup', 'config', 'robot.yaml'
    )
    vision_config = _share(
        'robot_vision', 'config', 'vision.yaml'
    )
    follower_config = _share(
        'robot_follower', 'config', 'follower.yaml'
    )
    safety_config = _share(
        'robot_safety', 'config', 'safety.yaml'
    )

    for path in (
        robot_config,
        vision_config,
        follower_config,
        safety_config,
    ):
        if not os.path.isfile(path):
            raise RuntimeError(
                f'Missing installed configuration: {path}'
            )

    clock_parameters = {'use_sim_time': use_sim_time}
    auto_topic = wiring['topics']['cmd_vel_out']

    def node(
        package,
        executable,
        config,
        name=None,
        overrides=None,
        remappings=None,
    ):
        return Node(
            package=package,
            executable=executable,
            name=name or executable,
            output='screen',
            emulate_tty=True,
            parameters=[
                config,
                robot_config,
                clock_parameters,
                overrides or {},
            ],
            remappings=remappings or [],
        )

    safety_manager = node(
        'robot_safety',
        'safety_manager_node',
        safety_config,
        remappings=[
            ('/cmd_vel_auto', auto_topic),
        ],
    )

    lidar_obstacle = node(
        'robot_safety',
        'lidar_obstacle_node',
        safety_config,
        overrides={
            'scan_topic': LaunchConfiguration(
                'scan_topic'
            ).perform(context),
        },
    )

    # 안전 프로세스 종료 시 전체 launch 종료를 요청한다.
    # 실제 모터 통신 단절 정지는 별도 MCU watchdog이 담당해야 한다.
    handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=action,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason=f'{name} exited'
                        )
                    ),
                ],
            )
        )
        for action, name in (
            (safety_manager, 'safety_manager_node'),
            (lidar_obstacle, 'lidar_obstacle_node'),
        )
    ]

    actions = [
        LogInfo(
            msg=(
                f'[Master] nav_mode={mode} '
                f'sim={sim} '
                f'use_sim_time={use_sim_time} '
                f'motion_enabled={motion} '
                f'camera_2={second_camera} '
                f'sim_estop={sim_estop} '
                f'sim_amcl_init={sim_amcl_init} '
                f'rviz={rviz}'
            )
        ),
    ] + handlers + [
        safety_manager,
        lidar_obstacle,
    ]

    actions.append(
        _include(
            'robot_description',
            'description.launch.py',
            {
                'sim': str(sim).lower(),
                'use_sim_time': str(use_sim_time).lower(),
            },
        )
    )

    if sim:
        simulation_arguments = {
            'start_description': 'false',
            'cmd_vel_topic': '/cmd_vel_safe',
            'gui': LaunchConfiguration('gui'),
            'world': LaunchConfiguration('world'),
            'spawn_delay_sec': LaunchConfiguration('spawn_delay_sec'),
            'publish_estop': str(sim_estop).lower(),
        }

        for axis in ('x', 'y', 'z', 'yaw'):
            simulation_arguments[axis] = LaunchConfiguration(
                f'sim_{axis}'
            )

        actions.append(
            _include(
                'robot_sim',
                'sim.launch.py',
                simulation_arguments,
            )
        )

    else:
        actions.append(
            node(
                'robot_vision',
                'camera_node',
                vision_config,
            )
        )

        if second_camera:
            actions.append(
                node(
                    'robot_vision',
                    'camera_node',
                    vision_config,
                    name='camera_node_2',
                )
            )

    for executable in (
        'apriltag_node',
        'qr_node',
        'detector_node',
        'pose_node',
    ):
        actions.append(
            node(
                'robot_vision',
                executable,
                vision_config,
            )
        )

    follower_parameters = {
        'cmd_vel_topic': (
            '/cmd_vel_follower'
            if mode == 'navigation'
            else auto_topic
        ),
    }

    if mode == 'navigation':
        follower_parameters['enable_motion'] = False

    actions.append(
        node(
            'robot_follower',
            'waypoint_follower_node',
            follower_config,
            overrides=follower_parameters,
        )
    )

    if mode != 'none':
        navigation_arguments = {
            name: LaunchConfiguration(name)
            for name in (
                'use_sim_time',
                'autostart',
                'bond_timeout',
                'set_initial_pose',
                'initial_pose_x',
                'initial_pose_y',
                'initial_pose_z',
                'initial_pose_yaw',
                'scan_topic',
                'wheel_odom_topic',
                'imu_topic',
                'odom_topic',
                'map_topic',
            )
        }

        navigation_arguments['publish_sensor_tf'] = 'false'

        if mode in ('localization', 'navigation'):
            navigation_arguments.update({
                'map': LaunchConfiguration('map'),
                'amcl_params_file': LaunchConfiguration(
                    'amcl_params_file'
                ),
            })

        if sim and sim_amcl_init and mode in ('localization', 'navigation'):
            # Gazebo 스폰 위치 = map 좌표라는 가정(지도를 기본 스폰 위치에서 만든 경우).
            # 2D Pose Estimate 를 누를 필요가 없고, 목표 실행 중 위치 재설정도 필요 없다.
            navigation_arguments.update({
                'set_initial_pose': 'true',
                'initial_pose_x': LaunchConfiguration('sim_x'),
                'initial_pose_y': LaunchConfiguration('sim_y'),
                'initial_pose_z': '0.0',
                'initial_pose_yaw': LaunchConfiguration('sim_yaw'),
            })

            actions.append(
                LogInfo(
                    msg=(
                        '[Master] sim_amcl_init: AMCL initial pose = Gazebo '
                        'spawn pose (assumes the map was recorded from the '
                        'default spawn pose; otherwise use '
                        'sim_amcl_init:=false and RViz 2D Pose Estimate)'
                    )
                )
            )

        if mode == 'navigation':
            navigation_arguments.update({
                'motion_enabled': str(motion).lower(),
                'cmd_vel_out_topic': auto_topic,
                'cmd_vel_dry_run_topic': wiring['topics'][
                    'cmd_vel_dry_run'
                ],
                'nav2_params_file': LaunchConfiguration(
                    'nav2_params_file'
                ),
            })

        actions.append(
            _include(
                'robot_navigation',
                f'{mode}.launch.py',
                navigation_arguments,
            )
        )

    if rviz:
        # RViz 도 use_sim_time 을 맞춰야 /initialpose stamp 와 TF 시각이 어긋나지 않는다.
        rviz_arguments = []

        try:
            rviz_config = _share(
                'nav2_bringup', 'rviz', 'nav2_default_view.rviz'
            )
        except PackageNotFoundError:
            rviz_config = ''

        if rviz_config and os.path.isfile(rviz_config):
            rviz_arguments = ['-d', rviz_config]

        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=rviz_arguments,
                parameters=[clock_parameters],
            )
        )

    return actions


def generate_launch_description():
    wiring = load_wiring()

    arguments = [
        DeclareLaunchArgument(
            'nav_mode',
            default_value='none',
            description='none | mapping | localization | navigation',
        ),
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Start Gazebo',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=LaunchConfiguration('sim'),
            description='Use /clock; defaults to sim',
        ),
        DeclareLaunchArgument(
            'motion_enabled',
            default_value='false',
            description='Allow Nav2 output to reach robot_safety',
        ),
        DeclareLaunchArgument(
            'enable_camera_2',
            default_value='false',
            description='Start camera_node_2 outside simulation',
        ),
        DeclareLaunchArgument(
            'sim_cmd_vel_topic',
            default_value='/cmd_vel_safe',
            description='Master simulation requires /cmd_vel_safe',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_navigation'),
                'maps',
                'exhibition_map.yaml',
            ]),
            description='Saved map YAML',
        ),
        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=_share(
                'robot_navigation', 'config', 'amcl.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=_share(
                'robot_navigation', 'config', 'nav2.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Gazebo GUI',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_sim'),
                'worlds',
                'exhibition.sdf',
            ]),
        ),
        DeclareLaunchArgument(
            'spawn_delay_sec',
            default_value='4.0',
        ),
        DeclareLaunchArgument(
            'sim_estop',
            default_value=LaunchConfiguration('sim'),
            description=(
                'SIMULATION ONLY: robot_sim publishes /estop_state=false '
                'so robot_safety can leave STOP; defaults to sim, '
                'rejected when sim=false'
            ),
        ),
        DeclareLaunchArgument(
            'sim_amcl_init',
            default_value=LaunchConfiguration('sim'),
            description=(
                'Set the AMCL initial pose to the Gazebo spawn pose '
                '(sim_x/sim_y/sim_yaw); defaults to sim'
            ),
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz (nav2 default view) with use_sim_time',
        ),
    ]

    arguments += [
        argument
        for argument in declare_common_arguments(wiring)
        if argument.name != 'use_sim_time'
    ]

    for axis, default in (
        ('x', '0.0'),
        ('y', '0.0'),
        ('z', '0.10'),
        ('yaw', '0.0'),
    ):
        arguments.append(
            DeclareLaunchArgument(
                f'sim_{axis}',
                default_value=default,
            )
        )

    return LaunchDescription(
        arguments + [
            OpaqueFunction(
                function=_build_system,
                args=[wiring],
            ),
        ]
    )