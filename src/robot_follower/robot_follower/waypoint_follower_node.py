#!/usr/bin/env python3
"""
[Follower] waypoint_follower_node

Role
----
Receive a target (from vision now, from navigation later) and generate
the autonomous driving command.

    /vision/tag_pose   (geometry_msgs/PoseStamped) --+
    /vision/qr_result  (std_msgs/String)           --+--> waypoint_follower_node --> /cmd_vel_auto (geometry_msgs/Twist)
    /odom, /waypoints  (future)                    --+

Safety rules (do not remove)
----------------------------
1. The default command is ALWAYS linear.x = 0.0, angular.z = 0.0.
2. enable_motion = false (default) -> cmd_vel stays 0 no matter what vision publishes.
3. A target older than target_timeout_sec is treated as lost -> cmd_vel = 0.
4. This node NEVER talks to the STM32 / motors directly. It only publishes
   /cmd_vel_auto. The future chain is:
       /cmd_vel_auto -> robot_safety -> /cmd_vel_safe -> robot_control -> robot_bridge -> STM32

Status
------
SKELETON. compute_control() returns (0.0, 0.0). Fill in Pure Pursuit / PID /
waypoint following in the "TODO: USER IMPLEMENTATION" block.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

# ==================================================
# TOPIC NAMES
# Manage topic names here (or override them in follower.yaml).
# Do NOT hard-code topic strings anywhere else in this file.
# ==================================================
DEFAULT_TAG_POSE_TOPIC = '/vision/tag_pose'
DEFAULT_QR_RESULT_TOPIC = '/vision/qr_result'
DEFAULT_CMD_VEL_TOPIC = '/cmd_vel_auto'
# Future inputs (not used yet)
DEFAULT_ODOM_TOPIC = '/odom'
DEFAULT_WAYPOINTS_TOPIC = '/waypoints'


class WaypointFollowerNode(Node):
    """Waypoint / path follower (skeleton)."""

    def __init__(self):
        super().__init__('waypoint_follower_node')

        # --------------------------------------------------
        # Parameters
        # Defaults are declared here; follower.yaml / robot.yaml override them.
        # Numeric values are initial test values -> TODO: tune on the real robot.
        # --------------------------------------------------
        self.declare_parameter('enable_motion', False)       # SAFETY switch
        self.declare_parameter('max_linear_velocity', 0.3)   # [m/s]   TODO: tune
        self.declare_parameter('max_angular_velocity', 0.5)  # [rad/s] TODO: tune
        self.declare_parameter('goal_tolerance', 0.3)        # [m]     TODO: tune
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('target_timeout_sec', 1.0)
        self.declare_parameter('heartbeat_period_sec', 5.0)
        self.declare_parameter('tag_pose_topic', DEFAULT_TAG_POSE_TOPIC)
        self.declare_parameter('qr_result_topic', DEFAULT_QR_RESULT_TOPIC)
        self.declare_parameter('cmd_vel_topic', DEFAULT_CMD_VEL_TOPIC)

        self.enable_motion = self.get_parameter('enable_motion').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.control_rate_hz = self.get_parameter('control_rate_hz').value
        self.target_timeout_sec = self.get_parameter('target_timeout_sec').value
        self.heartbeat_period_sec = self.get_parameter('heartbeat_period_sec').value
        self.tag_pose_topic = self.get_parameter('tag_pose_topic').value
        self.qr_result_topic = self.get_parameter('qr_result_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # --------------------------------------------------
        # State
        # --------------------------------------------------
        self.target_pose = None            # geometry_msgs/PoseStamped (latest target)
        self.target_received_time = None   # rclpy.time.Time
        self.destination = None            # str (latest QR result)
        self.last_enable_motion = self.enable_motion
        self.last_cmd = (0.0, 0.0)

        # --------------------------------------------------
        # Publisher
        # --------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # --------------------------------------------------
        # Subscribers (current inputs)
        # --------------------------------------------------
        self.tag_pose_sub = self.create_subscription(
            PoseStamped, self.tag_pose_topic, self.update_target, 10)
        self.qr_result_sub = self.create_subscription(
            String, self.qr_result_topic, self.update_destination, 10)

        # ==================================================
        # TODO: Future inputs (uncomment when available)
        # ==================================================
        # from nav_msgs.msg import Odometry   (and add nav_msgs to package.xml)
        # self.odom_sub = self.create_subscription(
        #     Odometry, DEFAULT_ODOM_TOPIC, self.update_odom, 10)
        # self.waypoints_sub = self.create_subscription(
        #     <WaypointsMsg>, DEFAULT_WAYPOINTS_TOPIC, self.update_waypoints, 10)

        # --------------------------------------------------
        # Timers
        # --------------------------------------------------
        control_period = 1.0 / self.control_rate_hz if self.control_rate_hz > 0.0 else 0.1
        self.control_timer = self.create_timer(control_period, self.control_loop)
        self.heartbeat_timer = self.create_timer(
            self.heartbeat_period_sec, self.heartbeat_callback)

        self.get_logger().info('[Follower] Waypoint follower started')
        self.get_logger().info(
            f'[Follower]   enable_motion={self.enable_motion} '
            f'(cmd_vel stays 0.0 until enable_motion=true) | '
            f'{self.tag_pose_topic} -> {self.cmd_vel_topic} @ {self.control_rate_hz} Hz')

    # --------------------------------------------------
    # Input callbacks
    # --------------------------------------------------
    def update_target(self, msg):
        """Store the latest target pose (from /vision/tag_pose)."""
        self.target_pose = msg
        self.target_received_time = self.get_clock().now()

    def update_destination(self, msg):
        """Store the latest destination info (from /vision/qr_result)."""
        if msg.data != self.destination:
            self.get_logger().info(f'[Follower] destination updated: "{msg.data}"')
        self.destination = msg.data
        # TODO: map destination -> waypoint list (robot_manager / database later)

    def has_valid_target(self):
        """True if a target exists and is not older than target_timeout_sec."""
        if self.target_pose is None or self.target_received_time is None:
            return False
        if self.target_timeout_sec <= 0.0:
            return True
        age_sec = (self.get_clock().now() - self.target_received_time).nanoseconds * 1e-9
        return age_sec <= self.target_timeout_sec

    # --------------------------------------------------
    # Control loop
    # --------------------------------------------------
    def control_loop(self):
        """Runs at control_rate_hz: compute and publish the command."""
        # enable_motion is re-read every cycle so it can be toggled at runtime:
        #   ros2 param set /waypoint_follower_node enable_motion true
        enable_motion = self.get_parameter('enable_motion').value
        if enable_motion != self.last_enable_motion:
            state = 'ENABLED' if enable_motion else 'DISABLED'
            self.get_logger().warn(f'[Follower] motion {state} (enable_motion={enable_motion})')
            self.last_enable_motion = enable_motion

        # SAFETY: the default command is always zero
        linear_velocity = 0.0
        angular_velocity = 0.0

        if enable_motion and self.has_valid_target():
            linear_velocity, angular_velocity = self.compute_control()
            linear_velocity = self.clamp(linear_velocity, self.max_linear_velocity)
            angular_velocity = self.clamp(angular_velocity, self.max_angular_velocity)

        self.publish_command(linear_velocity, angular_velocity)

    # ==================================================
    # TODO: USER IMPLEMENTATION
    # ==================================================
    def compute_control(self):
        """
        Compute the velocity command toward the current target.

        TODO:
        - Pure Pursuit
        - PID
        - Waypoint Following

        Inputs available:
            self.target_pose     geometry_msgs/PoseStamped (latest target, camera frame)
            self.destination     str or None (latest QR result)
            self.goal_tolerance  [m]
            # future: odometry, waypoint list

        Returns:
            (linear_velocity [m/s], angular_velocity [rad/s])
            The caller clamps both to max_linear_velocity / max_angular_velocity.
        """
        # Example of reading the target (uncomment when implementing):
        # import math   <- put at the top of the file
        # dx = self.target_pose.pose.position.x
        # dy = self.target_pose.pose.position.y
        # distance = math.hypot(dx, dy)
        # if distance < self.goal_tolerance:
        #     return 0.0, 0.0

        linear_velocity = 0.0
        angular_velocity = 0.0
        return linear_velocity, angular_velocity
    # ==================================================

    # --------------------------------------------------
    # Output
    # --------------------------------------------------
    def publish_command(self, linear_velocity, angular_velocity):
        """Publish /cmd_vel_auto. Never touches motors or STM32 directly."""
        msg = Twist()
        msg.linear.x = float(linear_velocity)
        msg.angular.z = float(angular_velocity)
        self.cmd_vel_pub.publish(msg)
        self.last_cmd = (msg.linear.x, msg.angular.z)

    @staticmethod
    def clamp(value, limit):
        """Limit value to [-limit, +limit]."""
        limit = abs(limit)
        return max(-limit, min(limit, value))

    def heartbeat_callback(self):
        """Low-rate status log so you can see the node is alive."""
        target_state = 'valid' if self.has_valid_target() else 'none'
        destination = self.destination if self.destination is not None else 'none'
        self.get_logger().info(
            f'[Follower] alive | enable_motion={self.last_enable_motion} | '
            f'target={target_state} | destination={destination} | '
            f'cmd=({self.last_cmd[0]:.2f}, {self.last_cmd[1]:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # try_shutdown(): same as rclpy.shutdown(), but safe if Ctrl+C already shut rclpy down
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
