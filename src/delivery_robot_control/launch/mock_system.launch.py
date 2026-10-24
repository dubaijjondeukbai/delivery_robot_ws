from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='delivery_robot_control',
            executable='control_subscriber',
            output='screen',
        ),
        Node(
            package='delivery_robot_control',
            executable='mock_stm32',
            output='screen',
        ),
        Node(
            package='delivery_robot_control',
            executable='mock_esp32',
            output='screen',
        ),
        Node(
            package='delivery_robot_follow',
            executable='mock_robot',
            output='screen',
        ),
        Node(
            package='delivery_robot_follow',
            executable='waypoint_follower',
            output='screen',
        ),
    ])