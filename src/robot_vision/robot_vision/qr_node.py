#!/usr/bin/env python3
"""
[Vision] qr_node

Role
----
Decode QR codes in camera images and publish the decoded text (destination / station info).

    /camera/image_raw (sensor_msgs/Image)
            |
            v
         qr_node  --->  /vision/qr_result (std_msgs/String, one message per newly seen text)

Status
------
IMPLEMENTED with cv2.QRCodeDetector (or cv2.QRCodeDetectorAruco when requested and
available; it is faster on small CPUs). detectAndDecodeMulti is used when the build
supports it and falls back to detectAndDecode when the multi variant is unavailable
or raises.

Repeat suppression
------------------
The same text is republished at most once every repeat_suppression_sec while it
stays in view, so a station marker that is visible for seconds at 15 fps does not
produce 15 messages per second. Empty decodes are never published.

Future
------
robot_manager / task_manager maps the decoded text (e.g. STATION_A) to a navigation
goal; robot_follower currently stores it as the destination.
"""

from __future__ import annotations

import time
from typing import Dict, List

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from robot_vision import common

# ==================================================
# TOPIC NAMES
# Defaults live in robot_vision/common.py (single source for every vision node)
# and can be overridden in vision.yaml. Do NOT hard-code topic strings anywhere
# else in this file.
# ==================================================
DEFAULT_IMAGE_TOPIC = common.TOPIC_IMAGE_RAW
DEFAULT_QR_RESULT_TOPIC = common.TOPIC_QR_RESULT

# After this many consecutive exceptions from detectAndDecodeMulti the node stops trying it.
MULTI_DECODE_MAX_FAILURES = 10


class QrDecoder:
    """Wraps the OpenCV QR detector and the multi -> single decode fallback."""

    def __init__(self, use_aruco_detector: bool, use_multi: bool) -> None:
        if use_aruco_detector and hasattr(cv2, 'QRCodeDetectorAruco'):
            self._detector = cv2.QRCodeDetectorAruco()
            self.backend = 'QRCodeDetectorAruco'
        else:
            self._detector = cv2.QRCodeDetector()
            self.backend = 'QRCodeDetector'
        self.multi_enabled = use_multi and hasattr(self._detector, 'detectAndDecodeMulti')
        self.multi_failures = 0

    def decode(self, gray) -> List[str]:
        """Return the non-empty texts decoded from the frame (may raise cv2.error)."""
        if self.multi_enabled:
            try:
                ok, decoded_info, _points, _straight = self._detector.detectAndDecodeMulti(gray)
                self.multi_failures = 0
                if not ok or not decoded_info:
                    return []
                return [text for text in decoded_info if text]
            except cv2.error:
                # Known to throw on some frames in a few OpenCV builds; fall back to the single decoder.
                self.multi_failures += 1
                if self.multi_failures >= MULTI_DECODE_MAX_FAILURES:
                    self.multi_enabled = False
        text, _points, _straight = self._detector.detectAndDecode(gray)
        return [text] if text else []


class QrNode(Node):
    """Decodes QR codes and publishes each text with repeat suppression."""

    def __init__(self) -> None:
        super().__init__('qr_node')

        self._bridge = CvBridge()

        # Heartbeat statistics.
        self._frames_received = 0
        self._frames_processed = 0
        self._detections = 0          # messages published
        self._suppressed = 0          # decodes dropped by repeat suppression
        self._conversion_failures = 0
        self._decode_failures = 0

        # text -> monotonic time of the last publish
        self._last_published: Dict[str, float] = {}

        self._declare_parameters()
        self._load_parameters()

        self._decoder = QrDecoder(self._use_aruco_detector, self._use_multi_decode)

        self._result_pub = self.create_publisher(String, self._qr_result_topic, 10)
        # Subscriber only when enabled; the publisher is always created so the topic
        # shows up in `ros2 topic list` either way.
        self._image_sub = None
        if self._enabled:
            self._image_sub = self.create_subscription(Image, self._image_topic, self._on_image,
                                                       qos_profile_sensor_data)
        else:
            self.get_logger().warning('[Vision] QR node is disabled (enabled=false): not subscribing to images')
        self._heartbeat_timer = self.create_timer(self._heartbeat_period_sec, self._on_heartbeat)

        self.get_logger().info(
            f'[Vision] qr_node ready: backend={self._decoder.backend}, multi_decode={self._decoder.multi_enabled}, '
            f'repeat_suppression_sec={self._repeat_suppression_sec:.1f}, '
            f'process_every_n_frames={self._process_every_n_frames}, image_topic={self._image_topic}')

    # --------------------------------------------------
    # Parameters
    # --------------------------------------------------
    def _declare_parameters(self) -> None:
        self.declare_parameter('enabled', True)             # false = heartbeat only, no image subscription
        self.declare_parameter('image_topic', DEFAULT_IMAGE_TOPIC)
        self.declare_parameter('qr_result_topic', DEFAULT_QR_RESULT_TOPIC)
        self.declare_parameter('repeat_suppression_sec', 1.0)
        # Decode only every Nth frame to save CPU (1 = every frame).
        self.declare_parameter('process_every_n_frames', 1)
        self.declare_parameter('use_multi_decode', True)
        self.declare_parameter('use_aruco_detector', False)
        self.declare_parameter('heartbeat_period_sec', 5.0)

    def _load_parameters(self) -> None:
        log = self.get_logger()
        p = self.get_parameter
        self._enabled = bool(p('enabled').value)
        self._image_topic = str(p('image_topic').value) or DEFAULT_IMAGE_TOPIC
        self._qr_result_topic = str(p('qr_result_topic').value) or DEFAULT_QR_RESULT_TOPIC
        suppression = p('repeat_suppression_sec').value
        try:
            self._repeat_suppression_sec = max(0.0, float(suppression))
        except (TypeError, ValueError):
            log.error(f"[Vision] Parameter 'repeat_suppression_sec'={suppression!r} is not a number; using 1.0")
            self._repeat_suppression_sec = 1.0
        self._process_every_n_frames = int(common.positive_or_default(
            log, 'process_every_n_frames', p('process_every_n_frames').value, 1))
        self._use_multi_decode = bool(p('use_multi_decode').value)
        self._use_aruco_detector = bool(p('use_aruco_detector').value)
        self._heartbeat_period_sec = float(common.positive_or_default(
            log, 'heartbeat_period_sec', p('heartbeat_period_sec').value, 5.0))

    # --------------------------------------------------
    # Image processing
    # --------------------------------------------------
    def _on_image(self, msg: Image) -> None:
        self._frames_received += 1
        if (self._frames_received - 1) % self._process_every_n_frames != 0:
            return
        self._frames_processed += 1

        try:
            gray = common.imgmsg_to_gray(self._bridge, msg)
        except (CvBridgeError, cv2.error) as exc:
            self._conversion_failures += 1
            self.get_logger().error(f'[Vision] cv_bridge conversion failed: {exc}', throttle_duration_sec=5.0)
            return

        try:
            texts = self._decoder.decode(gray)
        except cv2.error as exc:
            self._decode_failures += 1
            self.get_logger().warning(f'[Vision] QR decode failed: {exc}', throttle_duration_sec=5.0)
            return

        if not texts:
            return

        now = time.monotonic()
        for text in dict.fromkeys(texts):  # de-duplicate within the frame, keep order
            last = self._last_published.get(text)
            if last is not None and (now - last) < self._repeat_suppression_sec:
                self._suppressed += 1
                continue
            self._last_published[text] = now
            self._result_pub.publish(String(data=text))
            self._detections += 1
            self.get_logger().info(f'[Vision] QR decoded: {text!r}')

    # --------------------------------------------------
    # Heartbeat
    # --------------------------------------------------
    def _on_heartbeat(self) -> None:
        self._prune_suppression_table()
        self.get_logger().info(
            f'[Vision] QR alive | enabled={self._enabled} | frames_received={self._frames_received} | '
            f'frames_processed={self._frames_processed} | detections={self._detections} | '
            f'suppressed={self._suppressed} | decode_failures={self._decode_failures} | '
            f'conversion_failures={self._conversion_failures} | multi_decode={self._decoder.multi_enabled}')

    def _prune_suppression_table(self) -> None:
        """Drop texts not seen for a long time so the table cannot grow without bound."""
        horizon = max(60.0, self._repeat_suppression_sec * 10.0)
        cutoff = time.monotonic() - horizon
        for text in [t for t, stamp in self._last_published.items() if stamp < cutoff]:
            del self._last_published[text]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = QrNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:  # noqa: BLE001 - e.g. a parameter type error in vision.yaml
        get_logger('qr_node').fatal(f'[Vision] qr_node failed to start or crashed: {exc!r}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
