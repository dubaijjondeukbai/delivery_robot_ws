"""
4륜 조향 베이스 연결 노드 실행 (실물 전용).

    ros2 launch robot_base base.launch.py                       # config/base.yaml 의 port
    ros2 launch robot_base base.launch.py port:=/dev/ttyACM0
    ros2 launch robot_base base.launch.py mock:=true            # 가짜 STM32 와 함께(하드웨어 없이 시험)

robot.launch.py 와 따로 띄운다. 팀원의 stm32_serial_bridge 와 같은 포트를 쓰므로 함께 띄우지 않는다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

MOCK_PORT = '/tmp/ttySTM32'
# base.yaml 의 /** 공통값 -> 가짜 STM32 인자 (실물과 같은 바퀴 배치·조향 범위로 흉내 낸다)
MOCK_ARGUMENTS = {
    'wheel_x': '--wheel-x',
    'wheel_y': '--wheel-y',
    'wheel_radius': '--wheel-radius',
    'steer_mode': '--steer-mode',
    'max_steer_deg': '--max-steer-deg',
    'max_wheel_rpm': '--max-wheel-rpm',
    'steer_rate_dps': '--steer-rate-dps',
}


def _truthy(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _shared(config: str) -> dict:
    with open(config, encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    return (data.get('/**') or {}).get('ros__parameters') or {}


def _build(context):
    config = os.path.join(
        get_package_share_directory('robot_base'), 'config', 'base.yaml')
    mock = _truthy(LaunchConfiguration('mock').perform(context))
    port = LaunchConfiguration('port').perform(context).strip()
    overrides = {}
    actions = []

    if mock:
        if port and port != MOCK_PORT:
            raise RuntimeError(f'mock:=true uses {MOCK_PORT}; do not pass port')
        port = MOCK_PORT
        arguments = ['--link', MOCK_PORT]
        for name, flag in MOCK_ARGUMENTS.items():
            if name in _shared(config):
                arguments += [flag, str(_shared(config)[name])]
        actions.append(Node(
            package='robot_base',
            executable='mock_stm32',
            name='mock_stm32',
            output='screen',
            arguments=arguments,
        ))
    if port:
        overrides['port'] = port

    actions.append(LogInfo(msg=f"[robot_base] mock={mock} port={port or '(base.yaml)'}"))
    actions.append(Node(
        package='robot_base',
        executable='base_bridge_node',
        name='base_bridge_node',
        output='screen',
        emulate_tty=True,
        parameters=[config, overrides],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='', description='STM32 serial port'),
        DeclareLaunchArgument(
            'mock', default_value='false', description='Start the fake STM32 (pty) as well'),
        OpaqueFunction(function=_build),
    ])
