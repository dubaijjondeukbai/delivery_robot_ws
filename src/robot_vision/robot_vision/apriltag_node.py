#!/usr/bin/env python3
"""
[Vision] apriltag_node

Role
----
Detect AprilTags in camera images and publish the tag id and tag pose.

    /camera/image_raw (sensor_msgs/Image)
            |
            v
      apriltag_node  --->  /vision/tag_id    (std_msgs/String)
                     --->  /vision/tag_pose  (geometry_msgs/PoseStamped)

Status
------
SKELETON. The detection algorithm is NOT implemented yet.
Fill in detect_apriltag() in the "TODO: USER IMPLEMENTATION" block.
Until then nothing is published, but the node runs and prints a heartbeat.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

# ==================================================
# TOPIC NAMES
# Manage topic names here (or override them in vision.yaml).
# Do NOT hard-code topic strings anywhere else in this file.
# ==================================================
DEFAULT_IMAGE_TOPIC = '/camera/image_raw'
DEFAULT_TAG_ID_TOPIC = '/vision/tag_id'
DEFAULT_TAG_POSE_TOPIC = '/vision/tag_pose'


class AprilTagNode(Node):
    """AprilTag detector (skeleton)."""

    def __init__(self):
        super().__init__('apriltag_node')

        # --------------------------------------------------
        # Parameters
        # --------------------------------------------------
        self.declare_parameter('enabled', True)
        self.declare_parameter('tag_size', 0.15)             # [m] TODO: measure the real tag
        self.declare_parameter('heartbeat_period_sec', 5.0)
        self.declare_parameter('image_topic', DEFAULT_IMAGE_TOPIC)
        self.declare_parameter('tag_id_topic', DEFAULT_TAG_ID_TOPIC)
        self.declare_parameter('tag_pose_topic', DEFAULT_TAG_POSE_TOPIC)

        self.enabled = self.get_parameter('enabled').value
        self.tag_size = self.get_parameter('tag_size').value
        self.heartbeat_period_sec = self.get_parameter('heartbeat_period_sec').value
        self.image_topic = self.get_parameter('image_topic').value
        self.tag_id_topic = self.get_parameter('tag_id_topic').value
        self.tag_pose_topic = self.get_parameter('tag_pose_topic').value

        # --------------------------------------------------
        # Publishers
        # Always created, so the topics show up in `ros2 topic list`
        # even before detection is implemented.
        # --------------------------------------------------
        self.tag_id_pub = self.create_publisher(String, self.tag_id_topic, 10)
        self.tag_pose_pub = self.create_publisher(PoseStamped, self.tag_pose_topic, 10)

        # --------------------------------------------------
        # Subscriber
        # qos_profile_sensor_data (best effort) matches camera_node and is
        # compatible with most real camera drivers.
        # --------------------------------------------------
        self.image_sub = None
        if self.enabled:
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self.image_callback, qos_profile_sensor_data)
        else:
            self.get_logger().warn(
                '[Vision] AprilTag node is disabled (enabled=false): not subscribing to images')

        # --------------------------------------------------
        # Heartbeat
        # --------------------------------------------------
        self.frame_count = 0
        self.detection_count = 0
        self.heartbeat_timer = self.create_timer(
            self.heartbeat_period_sec, self.heartbeat_callback)

        self.get_logger().info('[Vision] AprilTag node started')
        self.get_logger().info(
            f'[Vision]   enabled={self.enabled} tag_size={self.tag_size} m | '
            f'{self.image_topic} -> {self.tag_id_topic}, {self.tag_pose_topic}')

    def image_callback(self, image_msg):
        """Called for every incoming image."""
        self.frame_count += 1

        result = self.detect_apriltag(image_msg)
        if result is None:
            return  # no tag in this frame -> publish nothing

        tag_id, pose = result
        self.publish_detection(tag_id, pose, image_msg.header)

    # ==================================================
    # TODO: USER IMPLEMENTATION
    # ==================================================
    def detect_apriltag(self, image_msg):
        """
        Detect an AprilTag in one image.

        Args:
            image_msg (sensor_msgs.msg.Image): input image

        Returns:
            None                -> no tag detected
            (tag_id, pose)      -> tag_id : str
                                   pose   : geometry_msgs.msg.Pose
                                            (tag pose in the camera frame)
        """
        # TODO: Implement AprilTag detection
        #   1. image_msg -> OpenCV image      (cv_bridge.imgmsg_to_cv2)
        #   2. run the detector               (pupil_apriltags / apriltag / cv2.aruco)
        #   3. estimate the pose              (camera intrinsics + self.tag_size)
        #   4. return (str(tag_id), geometry_msgs.msg.Pose)
        return None
    # ==================================================

    def publish_detection(self, tag_id, pose, image_header):
        """Publish tag id and tag pose (pose is stamped with the image header)."""
        id_msg = String()
        id_msg.data = str(tag_id)
        self.tag_id_pub.publish(id_msg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = image_header.stamp
        pose_msg.header.frame_id = image_header.frame_id
        pose_msg.pose = pose
        self.tag_pose_pub.publish(pose_msg)

        self.detection_count += 1

    def heartbeat_callback(self):
        """Low-rate status log so you can see the node is alive."""
        self.get_logger().info(
            f'[Vision] AprilTag alive | frames_received={self.frame_count} '
            f'detections={self.detection_count}')


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagNode()
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
