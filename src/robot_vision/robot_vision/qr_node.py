#!/usr/bin/env python3
"""
[Vision] qr_node

Role
----
Detect QR codes in camera images and publish the decoded result.

    /camera/image_raw (sensor_msgs/Image)
            |
            v
         qr_node  --->  /vision/qr_result (std_msgs/String)

Status
------
SKELETON. The detection algorithm is NOT implemented yet.
Fill in detect_qr() in the "TODO: USER IMPLEMENTATION" block.
Until then nothing is published, but the node runs and prints a heartbeat.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

# ==================================================
# TOPIC NAMES
# Manage topic names here (or override them in vision.yaml).
# Do NOT hard-code topic strings anywhere else in this file.
# ==================================================
DEFAULT_IMAGE_TOPIC = '/camera/image_raw'
DEFAULT_QR_RESULT_TOPIC = '/vision/qr_result'


class QRNode(Node):
    """QR code detector (skeleton)."""

    def __init__(self):
        super().__init__('qr_node')

        # --------------------------------------------------
        # Parameters
        # --------------------------------------------------
        self.declare_parameter('enabled', True)
        self.declare_parameter('heartbeat_period_sec', 5.0)
        self.declare_parameter('image_topic', DEFAULT_IMAGE_TOPIC)
        self.declare_parameter('qr_result_topic', DEFAULT_QR_RESULT_TOPIC)

        self.enabled = self.get_parameter('enabled').value
        self.heartbeat_period_sec = self.get_parameter('heartbeat_period_sec').value
        self.image_topic = self.get_parameter('image_topic').value
        self.qr_result_topic = self.get_parameter('qr_result_topic').value

        # --------------------------------------------------
        # Publisher (always created so the topic is visible in `ros2 topic list`)
        # --------------------------------------------------
        self.qr_result_pub = self.create_publisher(String, self.qr_result_topic, 10)

        # --------------------------------------------------
        # Subscriber (best-effort QoS, same as camera_node)
        # --------------------------------------------------
        self.image_sub = None
        if self.enabled:
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self.image_callback, qos_profile_sensor_data)
        else:
            self.get_logger().warn(
                '[Vision] QR node is disabled (enabled=false): not subscribing to images')

        # --------------------------------------------------
        # Heartbeat
        # --------------------------------------------------
        self.frame_count = 0
        self.detection_count = 0
        self.heartbeat_timer = self.create_timer(
            self.heartbeat_period_sec, self.heartbeat_callback)

        self.get_logger().info('[Vision] QR node started')
        self.get_logger().info(
            f'[Vision]   enabled={self.enabled} | {self.image_topic} -> {self.qr_result_topic}')

    def image_callback(self, image_msg):
        """Called for every incoming image."""
        self.frame_count += 1

        qr_text = self.detect_qr(image_msg)
        if qr_text is None:
            return  # no QR code in this frame -> publish nothing

        self.publish_result(qr_text)

    # ==================================================
    # TODO: USER IMPLEMENTATION
    # ==================================================
    def detect_qr(self, image_msg):
        """
        Detect and decode a QR code in one image.

        Args:
            image_msg (sensor_msgs.msg.Image): input image

        Returns:
            None   -> no QR code detected
            str    -> decoded QR text (e.g. destination id)
        """
        # TODO: Implement QR detection
        #   1. image_msg -> OpenCV image      (cv_bridge.imgmsg_to_cv2)
        #   2. decode                         (cv2.QRCodeDetector / pyzbar)
        #   3. return the decoded string
        return None
    # ==================================================

    def publish_result(self, qr_text):
        """Publish the decoded QR text."""
        msg = String()
        msg.data = str(qr_text)
        self.qr_result_pub.publish(msg)
        self.detection_count += 1

    def heartbeat_callback(self):
        """Low-rate status log so you can see the node is alive."""
        self.get_logger().info(
            f'[Vision] QR alive | frames_received={self.frame_count} '
            f'detections={self.detection_count}')


def main(args=None):
    rclpy.init(args=args)
    node = QRNode()
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
