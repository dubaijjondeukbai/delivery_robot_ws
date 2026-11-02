"""
QR 배달 시연 실행 (robot.launch.py nav_mode:=navigation 과 robot_base base.launch.py 는 따로 띄운다).

    ros2 launch robot_bringup delivery_demo.launch.py esp32_port:=/dev/serial/by-id/<ESP32>  # 실물
    ros2 launch robot_bringup delivery_demo.launch.py mock_lid:=true sim:=true   # 시뮬 + 가짜 뚜껑

esp32_port 를 주면 팀원 패키지의 esp32_serial_bridge 를 그 포트로 함께 띄운다.
sim:=true 는 STM32 가 없으므로 DRIVE_STOPPED 없이 /odom 만으로 정지를 확인한다.
가짜 뚜껑은 실물처럼 리미트 스위치가 없다(열림/닫힘이 "확인 안 됨"으로 나온다).
mock_limit_switches:=true 로 스위치가 있는 경우를 시험할 수 있다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _build(context):
    config = os.path.join(
        get_package_share_directory('robot_bringup'), 'config', 'delivery_demo.yaml')
    sim = _truthy(LaunchConfiguration('sim').perform(context))
    mock_lid = _truthy(LaunchConfiguration('mock_lid').perform(context))
    esp32_port = LaunchConfiguration('esp32_port').perform(context).strip()
    if mock_lid and esp32_port:
        raise RuntimeError('mock_lid:=true 와 esp32_port 는 함께 쓸 수 없습니다')

    overrides = {'use_sim_time': sim}
    drive_stopped = LaunchConfiguration('require_drive_stopped').perform(context).strip()
    if drive_stopped:
        overrides['require_drive_stopped'] = _truthy(drive_stopped)
    if sim:
        overrides['require_drive_stopped'] = False
    actions = [
        LogInfo(msg=f'[delivery_demo] sim={sim} mock_lid={mock_lid} '
                    f"esp32_port={esp32_port or '-'}"),
        Node(package='robot_bringup', executable='delivery_demo', name='delivery_demo',
             output='screen', emulate_tty=True, parameters=[config, overrides]),
    ]
    if mock_lid:
        switches = _truthy(LaunchConfiguration('mock_limit_switches').perform(context))
        actions.append(Node(package='robot_bringup', executable='mock_lid_esp32',
                            name='mock_lid_esp32', output='screen',
                            parameters=[{'limit_switches': switches}]))
    if esp32_port:
        actions.append(Node(package='delivery_robot_bridge', executable='esp32_serial_bridge',
                            name='esp32_serial_bridge', output='screen',
                            parameters=[{'port': esp32_port}],
                            remappings=[('esp32_command', '/esp32_command'),
                                        ('esp32_status', '/esp32_status')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('mock_lid', default_value='false',
                              description='가짜 ESP32 뚜껑 (펌웨어 없이 시험)'),
        DeclareLaunchArgument('require_drive_stopped', default_value='',
                              description='빈 값 = delivery_demo.yaml. '
                                          'false = STM32 DRIVE_STOPPED 없이 /odom 만으로 정지 확인'),
        DeclareLaunchArgument('mock_limit_switches', default_value='false',
                              description='가짜 ESP32 에 리미트 스위치가 있다고 볼지 (실물: 없음)'),
        DeclareLaunchArgument('esp32_port', default_value='',
                              description='실물 ESP32 시리얼 포트 (/dev/serial/by-id/... 권장)'),
        OpaqueFunction(function=_build),
    ])
