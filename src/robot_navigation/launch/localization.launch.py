"""
[Navigation] localization.launch.py

MODE B: localize in a saved map with AMCL.

    ros2 launch robot_navigation localization.launch.py map:=/home/<user>/maps/exhibition_map.yaml

    map.yaml/.pgm ─► map_server ──/map──┐
    LiDAR ──/scan───────────────────────┼─► amcl ──TF map->odom, /amcl_pose
    STM32 ──/wheel/odom──┐              │
    IMU ────/imu/data────┴─► ekf_filter_node ──/odom + TF odom->base_link
                                        ▲
                       lifecycle_manager_localization (map_server, amcl)

Result: the full TF chain map -> odom -> base_link, i.e. the robot pose in the
map frame. Nothing in this launch commands motion.

AMCL needs an initial pose: RViz "2D Pose Estimate", a /initialpose message, or
amcl.yaml set_initial_pose: true (see README.md).

Launch arguments (defaults from config/localization.yaml unless noted):
    map (default: <share>/maps/exhibition_map.yaml), amcl_params_file,
    scan_topic, wheel_odom_topic, imu_topic, odom_topic, map_topic,
    publish_sensor_tf, use_sim_time, autostart
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from robot_navigation.launch_common import (
    PACKAGE_NAME,
    config_path,
    declare_common_arguments,
    ekf_node,
    lifecycle_manager,
    load_wiring,
    sensor_remappings,
    static_sensor_tf_nodes,
)


def generate_launch_description() -> LaunchDescription:
    wiring = load_wiring()

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    amcl_params_file = LaunchConfiguration('amcl_params_file')
    # ParameterValue(..., str) keeps a path that looks numeric (e.g. "/maps/2026.yaml") a string.
    map_yaml = ParameterValue(LaunchConfiguration('map'), value_type=str)

    arguments = declare_common_arguments(wiring) + [
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([FindPackageShare(PACKAGE_NAME), 'maps', 'exhibition_map.yaml']),
            description='Full path of the map .yaml saved with map_saver_cli'),
        DeclareLaunchArgument(
            'amcl_params_file', default_value=config_path('amcl.yaml'),
            description='map_server + amcl parameter file'),
    ]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        # The dict after the file overrides yaml_filename ("" in amcl.yaml) with the launch argument.
        parameters=[amcl_params_file, {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
        remappings=sensor_remappings(),
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params_file, {'use_sim_time': use_sim_time}],
        remappings=sensor_remappings(),
    )

    return LaunchDescription(
        arguments
        + static_sensor_tf_nodes(wiring)
        + [
            ekf_node(use_sim_time),
            map_server,
            amcl,
            lifecycle_manager('lifecycle_manager_localization', ['map_server', 'amcl'], use_sim_time, autostart),
        ]
    )
