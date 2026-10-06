"""
[Sim] sim.launch.py

Gazebo (gz-sim) simulation of the delivery robot in the exhibition world.

    ros2 launch robot_sim sim.launch.py                         # GUI
    ros2 launch robot_sim sim.launch.py gui:=false              # headless server (WSL2 without GPU) + RViz elsewhere
    ros2 launch robot_sim sim.launch.py cmd_vel_topic:=/cmd_vel_auto   # drive straight from Nav2/follower (no robot_safety yet)

    gz sim (world + robot)  ──/scan, /imu/data, /camera/*, /wheel/odom, /joint_states, /clock──►  ROS 2
                            ◄──/model/delivery_robot/cmd_vel  (remapped from cmd_vel_topic)──────

Started here:
    1. robot_state_publisher with sim:=true URDF  (robot_description, unless start_description:=false)
    2. gz sim with worlds/<world>                   (ros_gz_sim gz_sim.launch.py)
    3. spawn of the robot from /robot_description    (ros_gz_sim create)
    4. ros_gz_bridge parameter_bridge               (config/bridge.yaml)
   (the spawn waits spawn_delay_sec, default 4 s, for the Gazebo server)

Normally this file is included by robot_bringup/robot.launch.py with sim:=true, which also
starts vision / follower / navigation with use_sim_time:=true and publish_sensor_tf:=false.

Safety: the simulated base listens to cmd_vel_topic (default /cmd_vel_safe = the robot_bridge input
on the real robot), so robot_safety keeps authority in simulation too. Use cmd_vel_topic:=/cmd_vel_auto
only for tests without robot_safety running.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('robot_sim')
    description_share = get_package_share_directory('robot_description')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    robot_name = LaunchConfiguration('robot_name')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------
    arguments = [
        DeclareLaunchArgument('world', default_value=os.path.join(sim_share, 'worlds', 'exhibition.sdf'),
                              description='SDF world file'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false = server only (headless), use RViz to look at the sensors'),
        DeclareLaunchArgument('robot_name', default_value='delivery_robot',
                              description='Model name in Gazebo (must match the /model/<name>/... topics in bridge.yaml)'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_safe',
                              description='ROS topic that drives the simulated base (robot_bridge input on the real robot)'),
        DeclareLaunchArgument('start_description', default_value='true',
                              description='Start robot_state_publisher here (false when robot_bringup already did)'),
        DeclareLaunchArgument('spawn_delay_sec', default_value='4.0',
                              description='Wait before spawning so the Gazebo server is up (WSL2 software rendering starts slowly)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.10'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    # Textures / meshes referenced relatively from the world file are found through this path.
    resource_path = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(sim_share, 'worlds'))

    # --------------------------------------------------
    # 1. Robot model + TF (URDF with Gazebo sensors/plugins)
    # --------------------------------------------------
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_share, 'launch', 'description.launch.py')),
        launch_arguments={'sim': 'true', 'use_sim_time': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('start_description')),
    )

    # --------------------------------------------------
    # 2. Gazebo server (+ GUI). '-r' starts the simulation running, '-s' = server only.
    # --------------------------------------------------
    server_only_flag = PythonExpression(["'-s ' if '", gui, "'.strip().lower() in ('false', '0') else ''"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [server_only_flag, '-r -v 2 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # --------------------------------------------------
    # 3. Spawn the robot from the latched /robot_description topic.
    #    Delayed by spawn_delay_sec: `create` gives up if the world service is not there yet,
    #    and a software-rendered Gazebo on WSL2 can take several seconds to come up.
    # --------------------------------------------------
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_delivery_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        parameters=[{'use_sim_time': True}],
    )

    # --------------------------------------------------
    # 4. gz <-> ROS bridge. The drive command's ROS side is remapped to cmd_vel_topic.
    # --------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(sim_share, 'config', 'bridge.yaml'),
            'use_sim_time': True,
        }],
        remappings=[('/model/delivery_robot/cmd_vel', cmd_vel_topic)],
    )

    ld = LaunchDescription()
    for argument in arguments:
        ld.add_action(argument)
    ld.add_action(resource_path)
    ld.add_action(description)
    ld.add_action(gazebo)
    ld.add_action(TimerAction(period=LaunchConfiguration('spawn_delay_sec'), actions=[spawn]))
    ld.add_action(bridge)
    return ld
