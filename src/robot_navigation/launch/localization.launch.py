"""저장된 지도에서 EKF + map_server + AMCL로 위치를 추정한다.

공통 인자는 launch_common.py가 localization.yaml에서 읽는다.
set_initial_pose와 initial_pose_*를 AMCL 파라미터로 연결한다.
센서 TF는 기본적으로 별도 robot_state_publisher가 제공한다.
이 launch는 주행 속도 명령을 발행하는 노드를 실행하지 않는다.
"""

import math
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from robot_navigation.launch_common import (
    PACKAGE_NAME,
    amcl_initial_pose_parameters,
    config_path,
    declare_common_arguments,
    ekf_node,
    lifecycle_manager,
    load_wiring,
    sensor_remappings,
    static_sensor_tf_nodes,
)


def _readable_file(value: str, argument: str) -> str:
    """실행 인자의 파일 경로를 확인하고 절대경로로 변환한다."""

    if not value.strip():
        raise RuntimeError(
            f'[Localization] {argument}: empty file path'
        )

    path = os.path.abspath(os.path.expanduser(value))

    if not os.path.isfile(path):
        raise RuntimeError(
            f'[Localization] {argument}: file not found: {path}. '
            f'Provide {argument}:=/absolute/path/to/file.yaml'
        )

    try:
        with open(path, 'rb') as handle:
            if not handle.read(1):
                raise RuntimeError(
                    f'[Localization] {argument}: empty file: {path}'
                )

    except OSError as exc:
        raise RuntimeError(
            f'[Localization] {argument}: cannot read {path}: {exc}'
        ) from exc

    return path


def _launch_localization(context, wiring):
    """launch 인자가 확정된 뒤 경로와 값을 확인하고 노드를 구성한다."""

    map_path = _readable_file(
        LaunchConfiguration('map').perform(context),
        'map',
    )

    amcl_path = _readable_file(
        LaunchConfiguration('amcl_params_file').perform(context),
        'amcl_params_file',
    )

    # launch 문자열을 ROS 파라미터의 실제 자료형으로 변환한다.
    try:
        use_sim_time = ParameterValue(
            LaunchConfiguration('use_sim_time'),
            value_type=bool,
        ).evaluate(context)

        autostart = ParameterValue(
            LaunchConfiguration('autostart'),
            value_type=bool,
        ).evaluate(context)

        bond_timeout = ParameterValue(
            LaunchConfiguration('bond_timeout'),
            value_type=float,
        ).evaluate(context)

    except (TypeError, ValueError) as exc:
        raise RuntimeError(
            '[Localization] Check use_sim_time, autostart '
            f'and bond_timeout: {exc}'
        ) from exc

    if not math.isfinite(bond_timeout) or bond_timeout < 0.0:
        raise RuntimeError(
            '[Localization] bond_timeout must be finite and >= 0.0'
        )

    # 12번에서 만든 함수를 통해 AMCL 초기 위치 파라미터 구성
    try:
        initial_parameters = {
            name: value.evaluate(context)
            for name, value in amcl_initial_pose_parameters().items()
        }

    except (TypeError, ValueError) as exc:
        raise RuntimeError(
            '[Localization] Check set_initial_pose '
            f'and initial_pose_*: {exc}'
        ) from exc

    for name, value in initial_parameters.items():
        if name.startswith('initial_pose.') and not math.isfinite(value):
            raise RuntimeError(
                f'[Localization] {name} must be finite'
            )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            amcl_path,
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': ParameterValue(
                    map_path,
                    value_type=str,
                ),
            },
        ],
        remappings=sensor_remappings(),
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_path,
            {
                'use_sim_time': use_sim_time,
                **initial_parameters,
            },
        ],
        remappings=sensor_remappings(),
    )

    initial_enabled = initial_parameters['set_initial_pose']

    message = (
        f'[Localization] map={map_path} '
        f'use_sim_time={use_sim_time} '
        f'autostart={autostart} '
        f'bond_timeout={bond_timeout} '
        f'set_initial_pose={initial_enabled}'
    )

    if initial_enabled:
        message += (
            f" initial_pose=(x={initial_parameters['initial_pose.x']},"
            f" y={initial_parameters['initial_pose.y']},"
            f" z={initial_parameters['initial_pose.z']},"
            f" yaw={initial_parameters['initial_pose.yaw']})"
        )
    else:
        message += (
            ' initial pose may be supplied through RViz /initialpose'
        )

    return (
        [LogInfo(msg=message)]
        + static_sensor_tf_nodes(wiring)
        + [
            ekf_node(use_sim_time),
            map_server,
            amcl,
            lifecycle_manager(
                'lifecycle_manager_localization',
                ['map_server', 'amcl'],
                use_sim_time,
                autostart,
            ),
        ]
    )


def generate_launch_description() -> LaunchDescription:
    wiring = load_wiring()

    arguments = declare_common_arguments(wiring) + [
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare(PACKAGE_NAME),
                'maps',
                'exhibition_map.yaml',
            ]),
            description=(
                'Map YAML path; checked when launching nodes'
            ),
        ),
        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=config_path('amcl.yaml'),
            description='Parameter file for map_server and AMCL',
        ),
    ]

    # --show-args는 인자만 표시한다.
    # 경로 검사와 노드 구성은 실제 실행 시 한다.
    return LaunchDescription(
        arguments + [
            OpaqueFunction(
                function=_launch_localization,
                args=[wiring],
            ),
        ]
    )