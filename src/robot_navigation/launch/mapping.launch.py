"""
[Navigation] mapping.launch.py

MODE A: build a 2D occupancy-grid map with slam_toolbox.

    ros2 launch robot_navigation mapping.launch.py

    LiDAR ──/scan──────────────┐
    STM32 ──/wheel/odom──┐     │
    IMU ────/imu/data────┼─► ekf_filter_node ──/odom + TF odom->base_link
                         │     │
                         └─────┴─► slam_toolbox (async) ──/map + TF map->odom
                                       ▲
                        configure + activate emitted by this launch file

slam_toolbox is a lifecycle node. It is driven directly from this launch file
(the same scheme as slam_toolbox's own online_async_launch.py): a CONFIGURE
transition is sent as soon as the node is up, and ACTIVATE is sent when the
node reports "inactive". use_lifecycle_manager is forced to false because the
bond that slam_toolbox creates towards nav2_lifecycle_manager raises
bad_weak_ptr during activation on some slam_toolbox releases (seen on Lyrical).

Nothing in this launch commands motion: drive the robot with the existing
teleop / follower path while mapping. Save the result with map_saver_cli
(see README.md, "지도 저장").

Launch arguments (defaults from config/localization.yaml):
    scan_topic, wheel_odom_topic, imu_topic, odom_topic, map_topic,
    publish_sensor_tf, use_sim_time, autostart (unused here), slam_params_file
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

from robot_navigation.launch_common import (
    config_path,
    declare_common_arguments,
    ekf_node,
    load_wiring,
    sensor_remappings,
    static_sensor_tf_nodes,
)


def generate_launch_description() -> LaunchDescription:
    wiring = load_wiring()

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    arguments = declare_common_arguments(wiring) + [
        DeclareLaunchArgument(
            'slam_params_file', default_value=config_path('slam.yaml'),
            description='slam_toolbox parameter file'),
    ]

    # Lifecycle node, transitions driven by the two actions below (no nav2 lifecycle manager, no bond).
    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',              # must match the top-level key in slam.yaml
        namespace='',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time, 'use_lifecycle_manager': False}],
        remappings=sensor_remappings(),   # scan/odom/map -> configured topics
    )

    # 1) CONFIGURE as soon as the node's lifecycle service is available (launch waits for it).
    configure_slam = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam_toolbox),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))

    # 2) ACTIVATE once the node reports the "inactive" state after configuring.
    activate_slam = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam_toolbox,
        start_state='configuring',
        goal_state='inactive',
        entities=[
            LogInfo(msg='[robot_navigation] slam_toolbox configured -> activating'),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_toolbox),
                transition_id=Transition.TRANSITION_ACTIVATE,
            )),
        ],
    ))

    return LaunchDescription(
        arguments
        + static_sensor_tf_nodes(wiring)
        + [
            ekf_node(use_sim_time),
            slam_toolbox,
            activate_slam,      # handler registered before the configure event is emitted
            configure_slam,
        ]
    )
