"""
[Navigation] navigation.launch.py

Localization (MODE B) + Nav2.

    # planning only, Nav2 output goes to /cmd_vel_nav_dry_run (nobody listens) — default
    ros2 launch robot_navigation navigation.launch.py map:=/home/<user>/maps/exhibition_map.yaml

    # after bench tests: route Nav2 to robot_safety
    ros2 launch robot_navigation navigation.launch.py map:=... motion_enabled:=true

    /goal_pose ─► bt_navigator ─► planner_server ─► path ─► controller_server ─► cmd_vel ─┐
                                                           behavior_server    ─► cmd_vel ─┤ (= cmd_vel_nav)
                                                                                           ▼
                                                                                 velocity_smoother
                                                                                           │ cmd_vel_smoothed
                     motion_enabled:=true  ─► /cmd_vel_auto ─► robot_safety ─► robot_control ─► robot_bridge ─► STM32
                     motion_enabled:=false ─► /cmd_vel_nav_dry_run (dead end)

SAFETY
  * Nav2 never publishes to the motor side directly; its only output is the remapped
    velocity_smoother topic above, and robot_safety keeps full authority downstream.
  * motion_enabled defaults to false and is NOT read from any config file: enabling real
    motion is an explicit per-run decision on the command line.
  * velocity_smoother publishes zero after velocity_timeout (1 s) without commands;
    controller_server publishes zero on goal end; a lost AMCL/TF makes the controller fail
    and stop publishing. robot_safety must still enforce its own command timeout.

Launch arguments (defaults from config/localization.yaml unless noted):
    map, motion_enabled (false), cmd_vel_out_topic, cmd_vel_dry_run_topic,
    nav2_params_file, amcl_params_file, scan_topic, wheel_odom_topic, imu_topic,
    odom_topic, map_topic, publish_sensor_tf, use_sim_time, autostart
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from robot_navigation.launch_common import (
    config_path,
    lifecycle_manager,
    load_wiring,
    package_share,
    sensor_remappings,
)

NAV2_LIFECYCLE_NODES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'velocity_smoother',
    'bt_navigator',
]


def generate_launch_description() -> LaunchDescription:
    wiring = load_wiring()
    topics = wiring['topics']

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    motion_enabled = LaunchConfiguration('motion_enabled')

    # Where the smoothed Nav2 command ends up. Evaluated at launch time:
    #   motion_enabled true  -> cmd_vel_out_topic   (/cmd_vel_auto -> robot_safety)
    #   anything else        -> cmd_vel_dry_run_topic (dead end, planning can be observed safely)
    nav2_cmd_vel_out = PythonExpression([
        "'", LaunchConfiguration('cmd_vel_out_topic'), "' if '", motion_enabled,
        "'.strip().lower() in ('true', '1') else '", LaunchConfiguration('cmd_vel_dry_run_topic'), "'",
    ])

    arguments = [
        # map / topic / use_sim_time / autostart arguments are declared by the included localization launch.
        DeclareLaunchArgument(
            'motion_enabled', default_value='false',
            description='true routes Nav2 velocity to cmd_vel_out_topic (robot_safety). '
                        'Keep false until bench tests are done. Never defaulted from a file.'),
        DeclareLaunchArgument(
            'cmd_vel_out_topic', default_value=topics['cmd_vel_out'],
            description='robot_safety input topic (geometry_msgs/Twist)'),
        DeclareLaunchArgument(
            'cmd_vel_dry_run_topic', default_value=topics['cmd_vel_dry_run'],
            description='Nav2 output while motion_enabled is false'),
        DeclareLaunchArgument(
            'nav2_params_file', default_value=config_path('nav2.yaml'),
            description='Nav2 parameter file'),
    ]

    # MODE B (EKF + map_server + AMCL). Its own arguments (map, scan_topic, ...) are forwarded
    # automatically because an included launch shares the launch configurations of its parent.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share(), 'launch', 'localization.launch.py')),
    )

    # Nav2 servers publish on the relative name "cmd_vel"; both are remapped to cmd_vel_nav so that
    # the velocity_smoother is the single exit point of the stack.
    nav2_to_smoother = [('cmd_vel', 'cmd_vel_nav')]

    controller_server = Node(
        package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=sensor_remappings() + nav2_to_smoother,
    )
    planner_server = Node(
        package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=sensor_remappings(),
    )
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=sensor_remappings() + nav2_to_smoother,
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=sensor_remappings(),
    )
    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=sensor_remappings() + [
            ('cmd_vel', 'cmd_vel_nav'),              # input: controller + behaviors
            ('cmd_vel_smoothed', nav2_cmd_vel_out),  # output: /cmd_vel_auto or the dry-run dead end
        ],
    )

    banner = LogInfo(msg=[
        '[robot_navigation] motion_enabled=', motion_enabled,
        ' -> Nav2 velocity is published on ', nav2_cmd_vel_out,
        ' (robot_safety listens on ', LaunchConfiguration('cmd_vel_out_topic'), ')',
    ])

    return LaunchDescription(
        arguments
        + [
            localization,
            banner,
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            velocity_smoother,
            lifecycle_manager('lifecycle_manager_navigation', NAV2_LIFECYCLE_NODES, use_sim_time, autostart),
        ]
    )
