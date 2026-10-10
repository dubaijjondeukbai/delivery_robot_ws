"""
[Master] robot.launch.py

Starts the whole delivery-robot system with one command:

    ros2 launch robot_bringup robot.launch.py

Nodes launched now:
    robot_description : robot_state_publisher (URDF -> TF base_link -> sensors), always
    robot_vision      : camera_node (real robot only), apriltag_node, qr_node, detector_node
    robot_follower    : waypoint_follower_node
    robot_navigation  : (optional, see nav_mode) EKF + slam_toolbox / map_server + AMCL / Nav2
    robot_sim         : (optional, sim:=true) Gazebo world + robot + ros_gz_bridge

Navigation modes (launch argument nav_mode, default none):
    nav_mode:=none          vision + follower only (same as the original skeleton)
    nav_mode:=mapping       + robot_navigation/mapping.launch.py       (EKF + slam_toolbox, drive with teleop/follower)
    nav_mode:=localization  + robot_navigation/localization.launch.py  (EKF + map_server + AMCL)
    nav_mode:=navigation    + robot_navigation/navigation.launch.py    (localization + Nav2)

    ros2 launch robot_bringup robot.launch.py nav_mode:=mapping
    ros2 launch robot_bringup robot.launch.py nav_mode:=navigation map:=$HOME/maps/exhibition_map.yaml
    ros2 launch robot_bringup robot.launch.py nav_mode:=navigation map:=... motion_enabled:=true   # after bench tests

Simulation (launch argument sim, default false):
    sim:=true   starts robot_sim (Gazebo), sets use_sim_time:=true for every node and replaces camera_node by
                the simulated camera (same /camera/image_raw + /camera/camera_info topics).
    ros2 launch robot_bringup robot.launch.py sim:=true nav_mode:=mapping
    ros2 launch robot_bringup robot.launch.py sim:=true nav_mode:=navigation map:=... motion_enabled:=true

Parameter files (later files override earlier ones):
    robot_vision/config/vision.yaml      -> vision nodes   (package defaults)
    robot_follower/config/follower.yaml  -> follower node  (package defaults)
    robot_bringup/config/robot.yaml      -> system-level overrides (loaded last)
    robot_navigation/config/*.yaml       -> navigation nodes (own chain; robot.yaml is NOT loaded there,
                                            use the launch arguments / localization.yaml instead)

Safety
    /cmd_vel_auto has exactly ONE publisher at a time:
        nav_mode != navigation : waypoint_follower_node (zeros until enable_motion=true)
        nav_mode == navigation : Nav2 velocity_smoother (only when motion_enabled:=true; otherwise a dead-end
                                 topic) and the follower is moved to /cmd_vel_follower.
    A future twist_mux / robot_manager will arbitrate between follower (docking) and Nav2.
    In simulation the base is driven by sim_cmd_vel_topic (default /cmd_vel_safe = robot_bridge input), so
    robot_safety keeps authority there too.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --------------------------------------------------
    # Launch arguments (navigation integration)
    # nav_mode       : none | mapping | localization | navigation (see module docstring)
    # map            : map yaml for localization / navigation
    # motion_enabled : SAFETY switch of navigation mode. false (default) keeps Nav2 velocity
    #                  on a dead-end topic; true routes it to /cmd_vel_auto -> robot_safety.
    #                  Never defaulted from a file: enabling motion is an explicit per-run decision.
    # --------------------------------------------------
    nav_mode = LaunchConfiguration('nav_mode')
    declare_nav_mode = DeclareLaunchArgument(
        'nav_mode', default_value='none',
        description='none | mapping | localization | navigation')
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution(
            [FindPackageShare('robot_navigation'), 'maps', 'exhibition_map.yaml']),
        description='Map yaml for nav_mode localization / navigation')
    declare_motion_enabled = DeclareLaunchArgument(
        'motion_enabled', default_value='false',
        description='navigation mode only: true routes Nav2 velocity to /cmd_vel_auto (robot_safety)')

    is_mapping = IfCondition(PythonExpression(["'", nav_mode, "' == 'mapping'"]))
    is_localization = IfCondition(PythonExpression(["'", nav_mode, "' == 'localization'"]))
    is_navigation = IfCondition(PythonExpression(["'", nav_mode, "' == 'navigation'"]))

    # --------------------------------------------------
    # Launch arguments (simulation)
    # sim               : true = Gazebo (robot_sim) instead of real sensors
    # use_sim_time      : defaults to the value of sim; passed to every node
    # sim_cmd_vel_topic : ROS topic that drives the simulated base. /cmd_vel_safe keeps the safety
    #                     chain intact; /cmd_vel_auto only for tests without robot_safety.
    # --------------------------------------------------
    sim = LaunchConfiguration('sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_sim = DeclareLaunchArgument(
        'sim', default_value='false',
        description='true: start robot_sim (Gazebo); the simulated camera replaces camera_node')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value=sim,
        description='Use /clock (defaults to the sim argument)')
    declare_sim_cmd_vel_topic = DeclareLaunchArgument(
        'sim_cmd_vel_topic', default_value='/cmd_vel_safe',
        description='Topic that drives the simulated base (robot_bridge input on the real robot)')
    # Appended after the yaml files for every node, so the launch argument wins over robot.yaml's /** entry.
    sim_time_param = {'use_sim_time': use_sim_time}

    # --------------------------------------------------
    # Parameter files
    # --------------------------------------------------
    robot_config = os.path.join(
        get_package_share_directory('robot_bringup'), 'config', 'robot.yaml')
    vision_config = os.path.join(
        get_package_share_directory('robot_vision'), 'config', 'vision.yaml')
    follower_config = os.path.join(
        get_package_share_directory('robot_follower'), 'config', 'follower.yaml')
    navigation_launch_dir = os.path.join(
        get_package_share_directory('robot_navigation'), 'launch')
    description_launch = os.path.join(
        get_package_share_directory('robot_description'), 'launch', 'description.launch.py')
    sim_launch = os.path.join(
        get_package_share_directory('robot_sim'), 'launch', 'sim.launch.py')

    # --------------------------------------------------
    # Robot model + TF (robot_description). Always on: the URDF is the single source of the
    # base_link -> sensor transforms, so robot_navigation runs with publish_sensor_tf:=false.
    # --------------------------------------------------
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch),
        launch_arguments={'sim': sim, 'use_sim_time': use_sim_time}.items(),
    )

    # --------------------------------------------------
    # Simulation (robot_sim): Gazebo world + robot + bridge. robot_state_publisher is already
    # started above, hence start_description:=false.
    # --------------------------------------------------
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'start_description': 'false',
            'cmd_vel_topic': LaunchConfiguration('sim_cmd_vel_topic'),
        }.items(),
        condition=IfCondition(sim),
    )

    # --------------------------------------------------
    # Vision nodes (robot_vision)
    # --------------------------------------------------
    # Real camera driver. In simulation the Gazebo camera publishes the same topics instead.
    camera_node = Node(
        package='robot_vision',
        executable='camera_node',
        name='camera_node',
        output='screen',
        emulate_tty=True,
        parameters=[vision_config, robot_config, sim_time_param],
        condition=UnlessCondition(sim),
    )

    apriltag_node = Node(
        package='robot_vision',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        emulate_tty=True,
        parameters=[vision_config, robot_config, sim_time_param],
    )

    qr_node = Node(
        package='robot_vision',
        executable='qr_node',
        name='qr_node',
        output='screen',
        emulate_tty=True,
        parameters=[vision_config, robot_config, sim_time_param],
    )

    # YOLO object detector. CPU heavy on the Pi 5 -> can be switched off system-wide with
    # detector_node.enabled in robot.yaml (the node then runs with a heartbeat only).
    detector_node = Node(
        package='robot_vision',
        executable='detector_node',
        name='detector_node',
        output='screen',
        emulate_tty=True,
        parameters=[vision_config, robot_config, sim_time_param],
    )

    # --------------------------------------------------
    # Follower node (robot_follower)
    # In navigation mode Nav2 owns /cmd_vel_auto, so the follower publishes to
    # /cmd_vel_follower instead (two publishers on the safety input would interleave
    # zeros with Nav2 commands). The override dict is loaded last, so it wins over
    # follower.yaml / robot.yaml.
    # --------------------------------------------------
    follower_cmd_vel_topic = PythonExpression(
        ["'/cmd_vel_follower' if '", nav_mode, "' == 'navigation' else '/cmd_vel_auto'"])
    waypoint_follower_node = Node(
        package='robot_follower',
        executable='waypoint_follower_node',
        name='waypoint_follower_node',
        output='screen',
        emulate_tty=True,
        parameters=[follower_config, robot_config, sim_time_param,
                    {'cmd_vel_topic': follower_cmd_vel_topic}],
    )

    # --------------------------------------------------
    # Navigation (robot_navigation) - only one of these is included, selected by nav_mode.
    # Their own launch arguments (scan_topic, wheel_odom_topic, ...) can be passed straight
    # through this launch, e.g. nav_mode:=mapping scan_topic:=/lidar/scan
    # publish_sensor_tf is false because robot_description already publishes base_link -> sensors.
    # --------------------------------------------------
    navigation_common_args = {
        'use_sim_time': use_sim_time,
        'publish_sensor_tf': 'false',
        'set_initial_pose': sim,   # sim: robot spawns at the map origin -> AMCL self-initialises, Nav2 comes up without RViz clicks
        # sim: WSL2 stalls of several seconds are normal -> disable bond monitoring; real robot: lenient 10 s
        'bond_timeout': PythonExpression(["'0.0' if '", sim, "'.strip().lower() in ('true', '1') else '10.0'"]),
    }
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_launch_dir, 'mapping.launch.py')),
        launch_arguments=navigation_common_args.items(),
        condition=is_mapping,
    )
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_launch_dir, 'localization.launch.py')),
        launch_arguments={**navigation_common_args, 'map': LaunchConfiguration('map')}.items(),
        condition=is_localization,
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_launch_dir, 'navigation.launch.py')),
        launch_arguments={
            **navigation_common_args,
            'map': LaunchConfiguration('map'),
            'motion_enabled': LaunchConfiguration('motion_enabled'),
        }.items(),
        condition=is_navigation,
    )

    # ==================================================
    # TODO Future Nodes
    # (create the package, then uncomment and add to the LaunchDescription)
    # ==================================================
    # safety_manager_node = Node(
    #     package='robot_safety',
    #     executable='safety_manager_node',
    #     name='safety_manager_node',
    #     output='screen',
    #     parameters=[robot_config],
    # )
    # drive_controller_node = Node(
    #     package='robot_control',
    #     executable='drive_controller_node',
    #     name='drive_controller_node',
    #     output='screen',
    #     parameters=[robot_config],
    # )
    # bridge_node = Node(
    #     package='robot_bridge',
    #     executable='bridge_node',
    #     name='bridge_node',
    #     output='screen',
    #     parameters=[robot_config],
    # )
    # ui_node = Node(
    #     package='robot_ui',
    #     executable='ui_node',
    #     name='ui_node',
    #     output='screen',
    # )
    # led_node = Node(
    #     package='robot_led',
    #     executable='led_node',
    #     name='led_node',
    #     output='screen',
    # )

    # --------------------------------------------------
    # Launch description
    # --------------------------------------------------
    ld = LaunchDescription()

    # Arguments
    ld.add_action(declare_nav_mode)
    ld.add_action(declare_map)
    ld.add_action(declare_motion_enabled)
    ld.add_action(declare_sim)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_sim_cmd_vel_topic)

    # Model / TF, Simulation
    ld.add_action(description)
    ld.add_action(simulation)

    # Vision
    ld.add_action(camera_node)
    ld.add_action(apriltag_node)
    ld.add_action(qr_node)
    ld.add_action(detector_node)

    # Follow
    ld.add_action(waypoint_follower_node)

    # Navigation (conditional)
    ld.add_action(mapping_launch)
    ld.add_action(localization_launch)
    ld.add_action(navigation_launch)

    # TODO Future Nodes
    # ld.add_action(safety_manager_node)
    # ld.add_action(drive_controller_node)
    # ld.add_action(bridge_node)
    # ld.add_action(ui_node)
    # ld.add_action(led_node)

    return ld
