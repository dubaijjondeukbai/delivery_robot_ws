"""
[Vision] common

Shared constants and helpers for the robot_vision package.

Topic names live here so every node agrees on them and a rename is a one-line
change. Each node additionally exposes its topic names as ROS parameters that
default to these constants. That keeps the perception nodes independent of a
particular camera driver: if another camera is wired in later, pointing
``image_topic`` at its topic in vision.yaml is enough.

This module deliberately imports no ROS packages so that the pure helpers can
be unit-tested without a ROS 2 installation.
"""

from __future__ import annotations

import math
from typing import Tuple

import cv2
import numpy as np

# ==================================================
# Topic names (single source of truth)
# ==================================================
TOPIC_IMAGE_RAW = '/camera/image_raw'
TOPIC_CAMERA_INFO = '/camera/camera_info'
TOPIC_TAG_ID = '/vision/tag_id'
TOPIC_TAG_POSE = '/vision/tag_pose'
TOPIC_QR_RESULT = '/vision/qr_result'
TOPIC_DETECTIONS = '/vision/detections'
TOPIC_DEBUG_IMAGE = '/vision/debug_image'              # YOLO annotated image (optional)
TOPIC_APRILTAG_DEBUG_IMAGE = '/vision/apriltag_debug_image'  # AprilTag overlay (optional)

# OpenCV / REP-103 optical frame: x right, y down, z forward (into the scene).
DEFAULT_CAMERA_FRAME_ID = 'camera_optical_frame'

# Image encodings that are already single-channel 8-bit.
GRAY_ENCODINGS = ('mono8', '8UC1')

# ==================================================
# Image conversion helpers (cv_bridge is passed in; keeps this module ROS-free)
# ==================================================


def imgmsg_to_bgr(bridge, msg) -> np.ndarray:
    """Convert a sensor_msgs/Image to an OpenCV BGR uint8 array."""
    return bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


def imgmsg_to_gray(bridge, msg) -> np.ndarray:
    """Convert a sensor_msgs/Image to a single-channel uint8 array.

    Mono images are passed through without a colour round trip; colour images
    are converted through bgr8 so that any colour encoding a future driver
    publishes (rgb8, bgra8, ...) is handled by cv_bridge.
    """
    if msg.encoding in GRAY_ENCODINGS:
        return bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
    bgr = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)


# ==================================================
# Geometry helpers
# ==================================================


def rotation_matrix_to_quaternion(rot: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a unit quaternion (x, y, z, w).

    Uses Shepperd's method (branch on the largest diagonal term) which is
    numerically stable for all rotations, including 180-degree ones.
    Implemented here to avoid a dependency on scipy / tf_transformations.
    """
    r = np.asarray(rot, dtype=np.float64)
    trace = r[0, 0] + r[1, 1] + r[2, 2]

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0 or not math.isfinite(norm):
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


# ==================================================
# Parameter validation helpers
# ==================================================


def positive_or_default(logger, name: str, value, default):
    """Return ``value`` if it is a finite number > 0, otherwise log and return ``default``.

    Used so that a typo in vision.yaml degrades to a sane default with a clear
    ERROR line instead of a timer with period 0 or a division by zero.
    """
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        logger.error(f"[Vision] Parameter '{name}'={value!r} is not a number; using default {default}")
        return default
    if not math.isfinite(numeric) or numeric <= 0.0:
        logger.error(f"[Vision] Parameter '{name}'={value!r} must be > 0; using default {default}")
        return default
    return value
