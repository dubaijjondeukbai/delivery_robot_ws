"""
[Master] robot.launch.py

Starts the whole delivery-robot system with one command:

    ros2 launch robot_bringup robot.launch.py

Nodes launched now:
    robot_vision   : camera_node, apriltag_node, qr_node
    robot_follower : waypoint_follower_node

Parameter files (later files override earlier ones):
    robot_vision/config/vision.yaml      -> vision nodes   (package defaults)
    robot_follower/config/follower.yaml  -> follower node  (package defaults)
    robot_bringup/config/robot.yaml      -> system-level overrides (loaded last)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # --------------------------------------------------
    # Parameter files
    # --------------------------------------------------
    robot_config = os.path.join(
        get_package_share_directory('robot_bringup'), 'config', 'robot.yaml')
    vision_config = os.path.join(
        get_package_share_directory('robot_vision'), 'config', 'vision.yaml')
    follower_config = os.path.join(
        get_package_share_directory('robot_follower'), 'config', 'follower.yaml')

    # --------------------------------------------------
    # Vision nodes (robot_vision)
    # --------------------------------------------------
    camera_node = Node(
        package='robot_vision',
        executable='camera_node',
        name='camera_node',
        output='screen',
        emulate_tty=True,
        parameters=[vision_config, robot_config],
    )

    apriltag_node = Node(
        package='robot_vision',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        emulate_tty=True,
        parameters=[vision_config, robot_config],
    )

    qr_node = Node(
        package='robot_vision',
        executable='qr_node',
        name='qr_node',
        output='screen',
        emulate_tty=True,
        parameters=[vision_config, robot_config],
    )

    # --------------------------------------------------
    # Follower node (robot_follower)
    # --------------------------------------------------
    waypoint_follower_node = Node(
        package='robot_follower',
        executable='waypoint_follower_node',
        name='waypoint_follower_node',
        output='screen',
        emulate_tty=True,
        parameters=[follower_config, robot_config],
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

    # Vision
    ld.add_action(camera_node)
    ld.add_action(apriltag_node)
    ld.add_action(qr_node)

    # Follow
    ld.add_action(waypoint_follower_node)

    # TODO Future Nodes
    # ld.add_action(safety_manager_node)
    # ld.add_action(drive_controller_node)
    # ld.add_action(bridge_node)
    # ld.add_action(ui_node)
    # ld.add_action(led_node)

    return ld
