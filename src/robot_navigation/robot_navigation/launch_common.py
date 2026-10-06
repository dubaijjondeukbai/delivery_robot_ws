"""
[Navigation] launch_common

Helpers shared by the robot_navigation launch files.

Plain functions, no wrapper classes. They read config/localization.yaml (the
"wiring" file: topic names, sensor mounting, launch defaults) and turn it into

  * launch-argument declarations whose defaults come from that file,
  * the topic remappings that connect the relative names used inside
    ekf.yaml / slam.yaml / amcl.yaml / nav2.yaml to the real robot topics,
  * the EKF node and the static base_link->sensor transforms that both the
    mapping and the localization launch need.

Keeping this in one module guarantees that mapping, localization and
navigation agree on every topic name.
"""

from __future__ import annotations

import os
from typing import Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = 'robot_navigation'
WIRING_FILE = 'localization.yaml'

REQUIRED_WIRING_KEYS = ('topics', 'frames', 'sensor_tf', 'defaults')
REQUIRED_TOPICS = ('scan', 'wheel_odom', 'imu', 'odom', 'map', 'cmd_vel_out', 'cmd_vel_dry_run')


def package_share() -> str:
    return get_package_share_directory(PACKAGE_NAME)


def config_path(file_name: str) -> str:
    """Absolute path of a file in the installed config/ directory."""
    return os.path.join(package_share(), 'config', file_name)


def load_wiring(path: str | None = None) -> Dict:
    """Read the wiring file and fail loudly on a missing section (a typo here must not become a silent no-op)."""
    path = path or config_path(WIRING_FILE)
    with open(path, 'r', encoding='utf-8') as handle:
        wiring = yaml.safe_load(handle) or {}
    missing = [key for key in REQUIRED_WIRING_KEYS if key not in wiring]
    if missing:
        raise RuntimeError(f'{path}: missing section(s) {missing}')
    missing_topics = [key for key in REQUIRED_TOPICS if key not in wiring['topics']]
    if missing_topics:
        raise RuntimeError(f'{path}: topics section is missing {missing_topics}')
    return wiring


def _bool_str(value) -> str:
    """launch arguments are strings: True -> 'true'."""
    return str(bool(value)).lower()


def declare_common_arguments(wiring: Dict) -> List[DeclareLaunchArgument]:
    """Arguments shared by all three launch files. Defaults come from the wiring file."""
    topics = wiring['topics']
    defaults = wiring['defaults']
    return [
        DeclareLaunchArgument(
            'use_sim_time', default_value=_bool_str(defaults.get('use_sim_time', False)),
            description='Use /clock (simulation) instead of wall time'),
        DeclareLaunchArgument(
            'autostart', default_value=_bool_str(defaults.get('autostart', True)),
            description='Lifecycle managers bring the managed nodes to ACTIVE automatically'),
        DeclareLaunchArgument('scan_topic', default_value=topics['scan'],
                              description='sensor_msgs/LaserScan topic'),
        DeclareLaunchArgument('wheel_odom_topic', default_value=topics['wheel_odom'],
                              description='Raw wheel odometry topic (nav_msgs/Odometry, no TF)'),
        DeclareLaunchArgument('imu_topic', default_value=topics['imu'],
                              description='sensor_msgs/Imu topic'),
        DeclareLaunchArgument('odom_topic', default_value=topics['odom'],
                              description='EKF output topic (nav_msgs/Odometry)'),
        DeclareLaunchArgument('map_topic', default_value=topics['map'],
                              description='nav_msgs/OccupancyGrid topic'),
        DeclareLaunchArgument(
            'publish_sensor_tf', default_value=_bool_str(wiring['sensor_tf'].get('publish', True)),
            description='Publish static base_link->laser_link / imu_link transforms from localization.yaml. '
                        'Set false once robot_bringup publishes a URDF.'),
    ]


def sensor_remappings() -> List:
    """Connect the relative names used in the YAML files to the configured topics.

    The /tf remaps are the Nav2 convention that keeps the stack namespace-ready; in the root
    namespace they are a no-op.
    """
    return [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('scan', LaunchConfiguration('scan_topic')),
        ('odom', LaunchConfiguration('odom_topic')),
        ('map', LaunchConfiguration('map_topic')),
    ]


def ekf_node(use_sim_time) -> Node:
    """robot_localization EKF: wheel odometry + IMU -> /odom and the odom->base_link TF."""
    return Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',      # must match the top-level key in ekf.yaml
        output='screen',
        parameters=[config_path('ekf.yaml'), {'use_sim_time': use_sim_time}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('wheel/odom', LaunchConfiguration('wheel_odom_topic')),      # ekf.yaml odom0
            ('imu/data', LaunchConfiguration('imu_topic')),               # ekf.yaml imu0
            ('odometry/filtered', LaunchConfiguration('odom_topic')),     # ekf_node output -> /odom
        ],
    )


def static_sensor_tf_nodes(wiring: Dict) -> List[Node]:
    """base_link -> laser_link and base_link -> imu_link from the sensor_tf section.

    Only started when the launch argument publish_sensor_tf is true. Uses the
    tf2_ros --x/--y/... argument style (Humble and newer).
    """
    frames = wiring['frames']
    sensor_tf = wiring['sensor_tf']
    nodes: List[Node] = []
    for sensor_key, frame_key in (('laser', 'laser'), ('imu', 'imu')):
        mount = sensor_tf[sensor_key]
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_base_to_{sensor_key}',
            output='screen',
            arguments=[
                '--x', str(float(mount.get('x', 0.0))),
                '--y', str(float(mount.get('y', 0.0))),
                '--z', str(float(mount.get('z', 0.0))),
                '--roll', str(float(mount.get('roll', 0.0))),
                '--pitch', str(float(mount.get('pitch', 0.0))),
                '--yaw', str(float(mount.get('yaw', 0.0))),
                '--frame-id', str(frames['base_link']),
                '--child-frame-id', str(frames[frame_key]),
            ],
            condition=IfCondition(LaunchConfiguration('publish_sensor_tf')),
        ))
    return nodes


def lifecycle_manager(name: str, node_names: List[str], use_sim_time, autostart) -> Node:
    """nav2_lifecycle_manager that configures/activates the given lifecycle nodes."""
    return Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name=name,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': node_names,
            # bond_timeout (default 4.0 s): raise to ~10.0 if the Pi logs "bond ... timed out" under load.
        }],
    )
