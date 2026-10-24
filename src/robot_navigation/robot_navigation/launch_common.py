"""robot_navigation의 설정 읽기와 공통 launch 구성 함수."""

from __future__ import annotations

import math
import os
import re
from typing import Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = 'robot_navigation'
WIRING_FILE = 'localization.yaml'

REQUIRED_WIRING_KEYS = (
    'topics',
    'frames',
    'sensor_tf',
    'defaults',
)

REQUIRED_TOPICS = (
    'scan',
    'wheel_odom',
    'imu',
    'odom',
    'map',
    'cmd_vel_out',
    'cmd_vel_dry_run',
)

REQUIRED_FRAMES = (
    'map',
    'odom',
    'base_link',
    'laser',
    'imu',
)

MOUNT_FIELDS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
POSE_FIELDS = ('x', 'y', 'z', 'yaw')

TOPIC_PATTERN = re.compile(
    r'/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*'
)


def package_share() -> str:
    return get_package_share_directory(PACKAGE_NAME)


def config_path(file_name: str) -> str:
    return os.path.join(package_share(), 'config', file_name)


def _mapping(value, label: str) -> Dict:
    if not isinstance(value, dict):
        raise RuntimeError(
            f'{label}: expected a YAML mapping'
        )
    return value


def _text(value, label: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or any(c.isspace() for c in value)
    ):
        raise RuntimeError(
            f'{label}: expected nonempty text without whitespace'
        )
    return value


def _boolean(value, label: str) -> bool:
    if type(value) is not bool:
        raise RuntimeError(
            f'{label}: use YAML true/false without quotes'
        )
    return value


def _number(value, label: str, minimum=None) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
    ):
        raise RuntimeError(
            f'{label}: expected a number'
        )

    result = float(value)

    if (
        not math.isfinite(result)
        or (minimum is not None and result < minimum)
    ):
        raise RuntimeError(
            f'{label}: invalid numeric value {value!r}'
        )

    return result


def load_wiring(path: str | None = None) -> Dict:
    """localization.yaml을 읽고 설정 형식과 범위를 검사한다."""

    path = path or config_path(WIRING_FILE)

    try:
        with open(path, 'r', encoding='utf-8') as handle:
            wiring = yaml.safe_load(handle)

    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(
            f'{path}: cannot read wiring YAML: {exc}'
        ) from exc

    wiring = _mapping(wiring, str(path))

    # 필수 설정 구역 확인
    for key in REQUIRED_WIRING_KEYS:
        if key not in wiring:
            raise RuntimeError(
                f'{path}: missing section {key!r}'
            )

        _mapping(wiring[key], f'{path}: {key}')

    # 토픽 이름 확인
    topics = wiring['topics']

    for key in REQUIRED_TOPICS:
        value = _text(
            topics.get(key),
            f'{path}: topics.{key}',
        )

        if TOPIC_PATTERN.fullmatch(value) is None:
            raise RuntimeError(
                f'{path}: topics.{key}: '
                'expected an absolute ROS topic name'
            )

    # 입력/출력 토픽이 실수로 같은 이름이 되는 경우 차단
    if (
        len({topics[key] for key in REQUIRED_TOPICS})
        != len(REQUIRED_TOPICS)
    ):
        raise RuntimeError(
            f'{path}: required input/output topics must be distinct'
        )

    # 프레임 이름 확인
    frames = wiring['frames']

    for key in REQUIRED_FRAMES:
        value = _text(
            frames.get(key),
            f'{path}: frames.{key}',
        )

        if (
            value.startswith('/')
            or value.endswith('/')
            or '//' in value
        ):
            raise RuntimeError(
                f'{path}: frames.{key}: '
                f'invalid frame name {value!r}'
            )

    if (
        len({frames[key] for key in REQUIRED_FRAMES})
        != len(REQUIRED_FRAMES)
    ):
        raise RuntimeError(
            f'{path}: map/odom/base/sensor frames must be distinct'
        )

    # 예비 센서 TF 설정 확인
    sensor_tf = wiring['sensor_tf']

    sensor_tf['publish'] = _boolean(
        sensor_tf.get('publish', False),
        f'{path}: sensor_tf.publish',
    )

    for sensor in ('laser', 'imu'):
        mount = _mapping(
            sensor_tf.get(sensor),
            f'{path}: sensor_tf.{sensor}',
        )

        for field in MOUNT_FIELDS:
            mount[field] = _number(
                mount.get(field),
                f'{path}: sensor_tf.{sensor}.{field}',
            )

    # launch 기본값 확인
    defaults = wiring['defaults']

    for key, fallback in (
        ('use_sim_time', False),
        ('autostart', True),
        ('set_initial_pose', False),
    ):
        defaults[key] = _boolean(
            defaults.get(key, fallback),
            f'{path}: defaults.{key}',
        )

    defaults['bond_timeout'] = _number(
        defaults.get('bond_timeout', 10.0),
        f'{path}: defaults.bond_timeout',
        minimum=0.0,
    )

    # AMCL 초기 위치 기본값
    pose = _mapping(
        defaults.get('initial_pose', {}),
        f'{path}: defaults.initial_pose',
    )

    for field in POSE_FIELDS:
        pose[field] = _number(
            pose.get(field, 0.0),
            f'{path}: defaults.initial_pose.{field}',
        )

    defaults['initial_pose'] = pose

    return wiring


def _bool_str(value) -> str:
    # 문자열 'false'가 bool 변환에서 True가 되는 문제를 막는다.
    return (
        'true'
        if _boolean(value, 'boolean launch default')
        else 'false'
    )


def declare_common_arguments(
    wiring: Dict,
) -> List[DeclareLaunchArgument]:
    """mapping/localization이 선언하고 navigation도 include로 공유한다."""

    topics = wiring['topics']
    defaults = wiring['defaults']

    arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=_bool_str(defaults['use_sim_time']),
            description='Use the simulation /clock',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value=_bool_str(defaults['autostart']),
            description='Automatically activate managed lifecycle nodes',
        ),
        DeclareLaunchArgument(
            'publish_sensor_tf',
            default_value=_bool_str(wiring['sensor_tf']['publish']),
            description=(
                'Publish fallback sensor TF; '
                'use false with robot_state_publisher'
            ),
        ),
        DeclareLaunchArgument(
            'bond_timeout',
            default_value=str(defaults['bond_timeout']),
            description=(
                'Lifecycle bond timeout in seconds; '
                '0.0 disables bond monitoring'
            ),
        ),
        DeclareLaunchArgument(
            'set_initial_pose',
            default_value=_bool_str(defaults['set_initial_pose']),
            description=(
                'Use the configured AMCL initial pose '
                '(requires localization launch wiring)'
            ),
        ),
    ]

    for argument, key in (
        ('scan_topic', 'scan'),
        ('wheel_odom_topic', 'wheel_odom'),
        ('imu_topic', 'imu'),
        ('odom_topic', 'odom'),
        ('map_topic', 'map'),
    ):
        arguments.append(
            DeclareLaunchArgument(
                argument,
                default_value=topics[key],
                description=f'ROS topic: {key}',
            )
        )

    for field in POSE_FIELDS:
        arguments.append(
            DeclareLaunchArgument(
                f'initial_pose_{field}',
                default_value=str(defaults['initial_pose'][field]),
                description=(
                    f'AMCL initial {field} in map frame '
                    '(metres; yaw in radians)'
                ),
            )
        )

    return arguments


def sensor_remappings() -> List:
    return [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('scan', LaunchConfiguration('scan_topic')),
        ('odom', LaunchConfiguration('odom_topic')),
        ('map', LaunchConfiguration('map_topic')),
    ]


def ekf_node(use_sim_time) -> Node:
    """바퀴 오도메트리와 IMU를 융합하고 odom -> base_link를 발행한다."""

    return Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            config_path('ekf.yaml'),
            {
                'use_sim_time': ParameterValue(
                    use_sim_time,
                    value_type=bool,
                ),
            },
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            (
                'wheel/odom',
                LaunchConfiguration('wheel_odom_topic'),
            ),
            (
                'imu/data',
                LaunchConfiguration('imu_topic'),
            ),
            (
                'odometry/filtered',
                LaunchConfiguration('odom_topic'),
            ),
        ],
    )


def static_sensor_tf_nodes(wiring: Dict) -> List[Node]:
    """publish_sensor_tf=true일 때만 사용하는 예비 센서 TF."""

    frames = wiring['frames']
    nodes = []

    for sensor in ('laser', 'imu'):
        mount = wiring['sensor_tf'][sensor]
        arguments = []

        for field in MOUNT_FIELDS:
            arguments.extend([
                f'--{field}',
                str(mount[field]),
            ])

        arguments.extend([
            '--frame-id',
            frames['base_link'],
            '--child-frame-id',
            frames[sensor],
        ])

        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tf_base_to_{sensor}',
                output='screen',
                arguments=arguments,
                parameters=[
                    {
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'),
                            value_type=bool,
                        ),
                    },
                ],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                ],
                condition=IfCondition(
                    LaunchConfiguration('publish_sensor_tf')
                ),
            )
        )

    return nodes


def amcl_initial_pose_parameters() -> Dict:
    """13번 localization.launch.py가 AMCL parameters에 추가할 때 적용된다."""

    parameters = {
        'set_initial_pose': ParameterValue(
            LaunchConfiguration('set_initial_pose'),
            value_type=bool,
        ),
    }

    for field in POSE_FIELDS:
        parameters[f'initial_pose.{field}'] = ParameterValue(
            LaunchConfiguration(f'initial_pose_{field}'),
            value_type=float,
        )

    return parameters


def lifecycle_manager(
    name: str,
    node_names: List[str],
    use_sim_time,
    autostart,
) -> Node:
    """기존 호출 형식을 유지하며 bond_timeout을 노드에 연결한다."""

    return Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name=name,
        output='screen',
        parameters=[
            {
                'use_sim_time': ParameterValue(
                    use_sim_time,
                    value_type=bool,
                ),
                'autostart': ParameterValue(
                    autostart,
                    value_type=bool,
                ),
                'node_names': list(node_names),
                'bond_timeout': ParameterValue(
                    LaunchConfiguration('bond_timeout'),
                    value_type=float,
                ),
            },
        ],
    )