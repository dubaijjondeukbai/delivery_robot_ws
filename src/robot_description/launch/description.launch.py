"""
[Description] description.launch.py

Publishes the robot model (URDF from urdf/robot.urdf.xacro) and its TF tree:

    xacro robot.urdf.xacro sim:=<sim>  ->  robot_state_publisher  ->  /robot_description (latched)
                                                                  ->  /tf_static : base_link -> laser_link / imu_link /
                                                                                   camera_link -> camera_optical_frame / base_footprint
                                                                  ->  /tf        : wheel joints (only when /joint_states arrive)

    ros2 launch robot_description description.launch.py                 # real robot (TF only)
    ros2 launch robot_description description.launch.py sim:=true       # adds Gazebo sensors/plugins to the URDF

Launch arguments:
    sim                   false | true   include gazebo.xacro (sensors + drive plugin) in the URDF
    use_sim_time          false | true   stamp TF with /clock
    publish_joint_states  false | true   start joint_state_publisher (zeros) so RViz shows the wheels on a bench
                                         without robot_bridge. Keep false in simulation (Gazebo publishes /joint_states).

Because this launch provides base_link -> sensor transforms, robot_navigation must run with
publish_sensor_tf:=false (robot_bringup/robot.launch.py does this).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim = LaunchConfiguration('sim')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_sim = DeclareLaunchArgument(
        'sim', default_value='false',
        description='true: include Gazebo sensors and drive plugin in the URDF')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock from the simulator')
    declare_publish_joint_states = DeclareLaunchArgument(
        'publish_joint_states', default_value='false',
        description='Run joint_state_publisher (bench/RViz only; Gazebo publishes /joint_states itself)')

    # --------------------------------------------------
    # URDF: xacro is expanded at launch time so the sim argument can switch gazebo.xacro on/off.
    # ParameterValue(..., str) keeps launch from trying to parse the XML as YAML.
    # --------------------------------------------------
    xacro_file = os.path.join(
        get_package_share_directory('robot_description'), 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim:=', sim]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('publish_joint_states')),
    )

    ld = LaunchDescription()
    ld.add_action(declare_sim)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_publish_joint_states)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    return ld
