#!/usr/bin/env python3
"""
[Vision] camera_node

Role
----
Manage the camera input and publish images.

    camera (hardware)  --->  camera_node  --->  /camera/image_raw (sensor_msgs/Image)

Modes
-----
use_camera = false  (default)
    MOCK MODE. No hardware access. A black bgr8 frame is published at
    publish_rate_hz so that apriltag_node / qr_node can be tested
    without a real camera.

use_camera = true
    REAL MODE. init_camera() / capture_frame() must be implemented
    (see the "TODO: USER IMPLEMENTATION" block). Until then the node keeps
    running but publishes nothing.

Future
------
OAK-D / USB camera / CSI camera driver integration.
"""

import array

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# ==================================================
# TOPIC NAMES
# Manage topic names here (or override them in vision.yaml).
# Do NOT hard-code topic strings anywhere else in this file.
# ==================================================
DEFAULT_IMAGE_TOPIC = '/camera/image_raw'


class CameraNode(Node):
    """Camera input manager. Mock mode by default."""

    def __init__(self):
        super().__init__('camera_node')

        # --------------------------------------------------
        # Parameters
        # Defaults are declared here; vision.yaml / robot.yaml override them.
        # --------------------------------------------------
        self.declare_parameter('use_camera', False)
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('publish_rate_hz', 5.0)       # TODO: tune (real camera: 15~30 Hz)
        self.declare_parameter('image_width', 320)           # TODO: real camera resolution
        self.declare_parameter('image_height', 240)          # TODO: real camera resolution
        self.declare_parameter('heartbeat_period_sec', 5.0)
        self.declare_parameter('image_topic', DEFAULT_IMAGE_TOPIC)

        self.use_camera = self.get_parameter('use_camera').value
        self.camera_id = self.get_parameter('camera_id').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.heartbeat_period_sec = self.get_parameter('heartbeat_period_sec').value
        self.image_topic = self.get_parameter('image_topic').value

        # --------------------------------------------------
        # Publisher
        # qos_profile_sensor_data (best effort) is the standard QoS for
        # camera streams. Subscribers in apriltag_node / qr_node use the same.
        # --------------------------------------------------
        self.image_pub = self.create_publisher(
            Image, self.image_topic, qos_profile_sensor_data)

        # --------------------------------------------------
        # Camera initialization
        # --------------------------------------------------
        self.camera_ready = False
        if self.use_camera:
            self.camera_ready = self.init_camera()
        else:
            self.get_logger().info(
                '[Vision] camera node running in mock mode (no hardware access)')

        # Mock frame: one black bgr8 image, created once and reused.
        # array.array('B') is accepted directly by sensor_msgs/Image.data.
        self.mock_frame_data = array.array(
            'B', bytes(self.image_width * self.image_height * 3))

        # --------------------------------------------------
        # Timers
        # --------------------------------------------------
        self.frame_count = 0
        publish_period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.2
        self.publish_timer = self.create_timer(publish_period, self.publish_callback)
        self.heartbeat_timer = self.create_timer(
            self.heartbeat_period_sec, self.heartbeat_callback)

        self.get_logger().info('[Vision] Camera node started')
        self.get_logger().info(
            f'[Vision]   use_camera={self.use_camera} camera_id={self.camera_id} '
            f'frame_id={self.frame_id} -> {self.image_topic} @ {self.publish_rate_hz} Hz')

    # ==================================================
    # TODO: USER IMPLEMENTATION
    # ==================================================
    def init_camera(self):
        """
        Open the real camera device.

        Returns:
            bool: True if the camera is ready, False otherwise.
        """
        # TODO: Camera driver integration
        #   USB camera : cv2.VideoCapture(self.camera_id)
        #   OAK-D      : depthai pipeline
        #   CSI camera : GStreamer pipeline
        self.get_logger().warn(
            '[Vision] use_camera=true but the camera driver is not implemented yet (TODO). '
            'No images will be published.')
        return False

    def capture_frame(self):
        """
        Grab one frame from the real camera and return it as sensor_msgs/Image.
        header.stamp / header.frame_id are filled by the caller.

        Returns:
            sensor_msgs.msg.Image, or None if no frame is available.
        """
        # TODO: Camera driver integration
        #   1. read a frame from the driver
        #   2. convert to sensor_msgs/Image (cv_bridge.cv2_to_imgmsg(frame, 'bgr8'))
        #   3. return the message
        return None
    # ==================================================

    def make_mock_frame(self):
        """Build a black bgr8 image message (mock mode)."""
        msg = Image()
        msg.height = self.image_height
        msg.width = self.image_width
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = self.image_width * 3
        msg.data = self.mock_frame_data
        return msg

    def publish_callback(self):
        """Capture (or mock) one frame and publish it."""
        if self.use_camera:
            if not self.camera_ready:
                return
            msg = self.capture_frame()
            if msg is None:
                return
        else:
            msg = self.make_mock_frame()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.image_pub.publish(msg)
        self.frame_count += 1

    def heartbeat_callback(self):
        """Low-rate status log so you can see the node is alive."""
        mode = 'real' if self.use_camera else 'mock'
        self.get_logger().info(
            f'[Vision] camera alive | mode={mode} | frames_published={self.frame_count}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
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
