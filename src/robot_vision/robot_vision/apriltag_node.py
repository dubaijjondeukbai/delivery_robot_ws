#!/usr/bin/env python3
"""
[Vision] apriltag_node

Role
----
Detect AprilTags (tag36h11) in camera images and publish the tag id and tag pose.

    /camera/image_raw   (sensor_msgs/Image)      --+
    /camera/camera_info (sensor_msgs/CameraInfo) --+--> apriltag_node --> /vision/tag_id   (std_msgs/String)
                                                                     --> /vision/tag_pose (geometry_msgs/PoseStamped)

Status
------
IMPLEMENTED with OpenCV's ArUco module (DICT_APRILTAG_36h11). Both ArUco APIs are
supported: cv2.aruco.ArucoDetector (OpenCV >= 4.7, ArUco in the main objdetect
module) and the legacy free functions (cv2.aruco.detectMarkers, OpenCV 4.5/4.6
from Ubuntu apt).

Pose
----
tag_pose is published ONLY when /camera/camera_info carries real intrinsics
(K[0] != 0). Without calibration the node still publishes tag_id and warns at a
throttled rate (calibration_warning_period_sec).

Pose convention
    The pose is the tag frame expressed in the camera optical frame
    (header.frame_id copied from the image). Tag frame = OpenCV IPPE_SQUARE
    convention: origin at the tag centre, x right, y up (as printed), z out of
    the tag towards the viewer. position.z is the distance along the optical axis
    in metres. A tag facing the camera squarely therefore has an orientation of
    ~180 deg about x (rvec ~ (pi, 0, 0), quaternion ~ (1, 0, 0, 0)); downstream
    code that wants the tag normal can use the tag frame's -z axis.

Corner ordering
    cv2.aruco returns the corners as TL, TR, BR, BL. cv2.SOLVEPNP_IPPE_SQUARE
    requires the object points in exactly that order:
        (-s/2, +s/2, 0), (+s/2, +s/2, 0), (+s/2, -s/2, 0), (-s/2, -s/2, 0)
    tag_size is the OUTER edge length of the black square in metres.

Target selection
    target_tag_id >= 0 : only that id is tracked
    target_tag_id == -1: the largest tag in view (the docking marker being approached)

Future
------
Station docking / final alignment (robot_follower or a docking node consumes
/vision/tag_pose). Not used for global localization (that is robot_navigation).
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from robot_vision import common

# ==================================================
# TOPIC NAMES
# Defaults live in robot_vision/common.py (single source for every vision node)
# and can be overridden in vision.yaml. Do NOT hard-code topic strings anywhere
# else in this file.
# ==================================================
DEFAULT_IMAGE_TOPIC = common.TOPIC_IMAGE_RAW
DEFAULT_CAMERA_INFO_TOPIC = common.TOPIC_CAMERA_INFO
DEFAULT_TAG_ID_TOPIC = common.TOPIC_TAG_ID
DEFAULT_TAG_POSE_TOPIC = common.TOPIC_TAG_POSE
DEFAULT_DEBUG_IMAGE_TOPIC = common.TOPIC_APRILTAG_DEBUG_IMAGE
DEFAULT_FRAME_ID = common.DEFAULT_CAMERA_FRAME_ID

Detection = Tuple[int, np.ndarray]  # (tag_id, corners (4, 2) float64 in TL, TR, BR, BL order)

# AprilTag family name (as in the AprilTag literature) -> cv2.aruco predefined dictionary name.
FAMILY_TO_DICT = {
    'tag36h11': 'DICT_APRILTAG_36h11',
    'tag36h10': 'DICT_APRILTAG_36h10',
    'tag25h9': 'DICT_APRILTAG_25h9',
    'tag16h5': 'DICT_APRILTAG_16h5',
}


class ArucoAprilTagDetector:
    """Thin wrapper that hides the OpenCV ArUco API differences.

    OpenCV >= 4.7 : cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(gray)
    OpenCV 4.5/4.6: cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    """

    def __init__(self, family: str) -> None:
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError(
                'cv2.aruco is not available in this OpenCV build. Install apt python3-opencv '
                '(Ubuntu builds include ArUco) or pip opencv-python>=4.7 / opencv-contrib-python.')
        dict_name = FAMILY_TO_DICT.get(family)
        if dict_name is None or not hasattr(cv2.aruco, dict_name):
            raise RuntimeError(
                f"Unsupported tag_family '{family}'. Supported: {sorted(FAMILY_TO_DICT)}")
        dict_id = getattr(cv2.aruco, dict_name)

        if hasattr(cv2.aruco, 'getPredefinedDictionary'):
            self._dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        else:  # very old API
            self._dictionary = cv2.aruco.Dictionary_get(dict_id)

        self._params = self._make_parameters()
        # Subpixel corner refinement tuned for AprilTags; improves pose accuracy noticeably.
        if hasattr(cv2.aruco, 'CORNER_REFINE_APRILTAG'):
            self._params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG

        if hasattr(cv2.aruco, 'ArucoDetector'):
            self._detector = cv2.aruco.ArucoDetector(self._dictionary, self._params)
            self.api = 'ArucoDetector (OpenCV >= 4.7)'
        else:
            self._detector = None
            self.api = 'legacy cv2.aruco.detectMarkers'

    @staticmethod
    def _make_parameters():
        # OpenCV >= 4.7 exposes a plain constructor; older builds only have the *_create() factory.
        if hasattr(cv2.aruco, 'DetectorParameters'):
            try:
                return cv2.aruco.DetectorParameters()
            except TypeError:
                pass
        return cv2.aruco.DetectorParameters_create()

    def detect(self, gray: np.ndarray) -> List[Detection]:
        """Return [(id, corners(4,2))] for every marker found in the grayscale image."""
        if self._detector is not None:
            corners, ids, _rejected = self._detector.detectMarkers(gray)
        else:
            corners, ids, _rejected = cv2.aruco.detectMarkers(gray, self._dictionary, parameters=self._params)
        if ids is None or len(ids) == 0:
            return []
        result: List[Detection] = []
        for marker_id, marker_corners in zip(ids, corners):
            result.append((int(np.asarray(marker_id).ravel()[0]),
                           np.asarray(marker_corners, dtype=np.float64).reshape(4, 2)))
        return result


def make_tag_object_points(tag_size: float) -> np.ndarray:
    """Object points of a square tag in the order required by cv2.SOLVEPNP_IPPE_SQUARE (TL, TR, BR, BL)."""
    h = float(tag_size) / 2.0
    return np.array([[-h, h, 0.0],
                     [h, h, 0.0],
                     [h, -h, 0.0],
                     [-h, -h, 0.0]], dtype=np.float64)


def corners_area(corners: np.ndarray) -> float:
    """Apparent image area of a tag; larger usually means closer to the camera."""
    return abs(float(cv2.contourArea(corners.astype(np.float32))))


class AprilTagNode(Node):
    """Detects AprilTags and publishes the selected tag's id and pose."""

    def __init__(self) -> None:
        super().__init__('apriltag_node')

        self._bridge = CvBridge()

        # Heartbeat statistics.
        self._frames_received = 0
        self._detections = 0
        self._poses_published = 0
        self._poses_rejected = 0
        self._conversion_failures = 0
        self._detection_failures = 0
        self._last_selected_id = None  # type: Optional[int]

        # Calibration state fed by /camera/camera_info.
        self._camera_matrix = None  # type: Optional[np.ndarray]
        self._dist_coeffs = None  # type: Optional[np.ndarray]
        self._calibrated = False
        self._camera_info_received = False
        self._camera_info_size = None  # type: Optional[Tuple[int, int]]

        self._declare_parameters()
        self._load_parameters()

        self._object_points = make_tag_object_points(self._tag_size)
        # IPPE_SQUARE (OpenCV >= 4.1) is the right solver for a planar square; fall back if missing.
        self._pnp_flags = getattr(cv2, 'SOLVEPNP_IPPE_SQUARE', cv2.SOLVEPNP_ITERATIVE)
        if self._pnp_flags == cv2.SOLVEPNP_ITERATIVE:
            self.get_logger().warning('[Vision] cv2.SOLVEPNP_IPPE_SQUARE not available; using SOLVEPNP_ITERATIVE')

        try:
            self._detector = ArucoAprilTagDetector(self._tag_family)  # type: Optional[ArucoAprilTagDetector]
            self.get_logger().info(f'[Vision] AprilTag detector ready: family={self._tag_family}, api={self._detector.api}')
        except (RuntimeError, AttributeError, cv2.error) as exc:
            self._detector = None
            self.get_logger().error(f'[Vision] AprilTag detector unavailable: {exc}. The node stays alive but '
                                    f'will not detect anything until this is fixed.')

        self._tag_id_pub = self.create_publisher(String, self._tag_id_topic, 10)
        self._tag_pose_pub = self.create_publisher(PoseStamped, self._tag_pose_topic, 10)
        self._debug_pub = None
        if self._publish_debug_image:
            self._debug_pub = self.create_publisher(Image, self._debug_image_topic, qos_profile_sensor_data)

        # Subscribers only when enabled; publishers are always created so the topics
        # show up in `ros2 topic list` either way.
        self._info_sub = None
        self._image_sub = None
        if self._enabled:
            self._info_sub = self.create_subscription(CameraInfo, self._camera_info_topic,
                                                      self._on_camera_info, qos_profile_sensor_data)
            self._image_sub = self.create_subscription(Image, self._image_topic,
                                                       self._on_image, qos_profile_sensor_data)
        else:
            self.get_logger().warning(
                '[Vision] AprilTag node is disabled (enabled=false): not subscribing to images')
        self._heartbeat_timer = self.create_timer(self._heartbeat_period_sec, self._on_heartbeat)

        target = 'largest visible tag' if self._target_tag_id < 0 else f'id {self._target_tag_id} only'
        self.get_logger().info(f'[Vision] apriltag_node ready: tag_size={self._tag_size:.3f} m, target={target}, '
                               f'image_topic={self._image_topic}')

    # --------------------------------------------------
    # Parameters
    # --------------------------------------------------
    def _declare_parameters(self) -> None:
        self.declare_parameter('enabled', True)             # false = heartbeat only, no image subscription
        self.declare_parameter('image_topic', DEFAULT_IMAGE_TOPIC)
        self.declare_parameter('camera_info_topic', DEFAULT_CAMERA_INFO_TOPIC)
        self.declare_parameter('tag_id_topic', DEFAULT_TAG_ID_TOPIC)
        self.declare_parameter('tag_pose_topic', DEFAULT_TAG_POSE_TOPIC)
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('tag_size', 0.15)          # metres, outer edge of the black square
        self.declare_parameter('target_tag_id', -1)       # -1 = largest tag in view
        self.declare_parameter('max_tag_distance_m', 10.0)  # sanity limit on position.z
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('debug_image_topic', DEFAULT_DEBUG_IMAGE_TOPIC)
        self.declare_parameter('calibration_warning_period_sec', 10.0)
        self.declare_parameter('heartbeat_period_sec', 5.0)

    def _load_parameters(self) -> None:
        log = self.get_logger()
        p = self.get_parameter
        self._enabled = bool(p('enabled').value)
        self._image_topic = str(p('image_topic').value) or DEFAULT_IMAGE_TOPIC
        self._camera_info_topic = str(p('camera_info_topic').value) or DEFAULT_CAMERA_INFO_TOPIC
        self._tag_id_topic = str(p('tag_id_topic').value) or DEFAULT_TAG_ID_TOPIC
        self._tag_pose_topic = str(p('tag_pose_topic').value) or DEFAULT_TAG_POSE_TOPIC
        self._tag_family = str(p('tag_family').value).strip().lower() or 'tag36h11'
        self._tag_size = float(common.positive_or_default(log, 'tag_size', p('tag_size').value, 0.15))
        self._target_tag_id = int(p('target_tag_id').value)
        self._max_tag_distance_m = float(common.positive_or_default(
            log, 'max_tag_distance_m', p('max_tag_distance_m').value, 10.0))
        self._publish_debug_image = bool(p('publish_debug_image').value)
        self._debug_image_topic = str(p('debug_image_topic').value) or DEFAULT_DEBUG_IMAGE_TOPIC
        self._calibration_warning_period_sec = float(common.positive_or_default(
            log, 'calibration_warning_period_sec', p('calibration_warning_period_sec').value, 10.0))
        self._heartbeat_period_sec = float(common.positive_or_default(
            log, 'heartbeat_period_sec', p('heartbeat_period_sec').value, 5.0))

    # --------------------------------------------------
    # CameraInfo
    # --------------------------------------------------
    def _on_camera_info(self, msg: CameraInfo) -> None:
        """Cache K and D. K[0] == 0 (sensor_msgs convention) means uncalibrated."""
        k = np.asarray(msg.k, dtype=np.float64).ravel()
        valid = k.size == 9 and bool(np.all(np.isfinite(k))) and k[0] > 0.0 and k[4] > 0.0

        if valid:
            self._camera_matrix = k.reshape(3, 3)
            d = np.asarray(msg.d, dtype=np.float64).ravel()
            if d.size == 0 or not np.all(np.isfinite(d)):
                d = np.zeros(5, dtype=np.float64)
            self._dist_coeffs = d.reshape(-1, 1)
            self._camera_info_size = (int(msg.width), int(msg.height))
        else:
            self._camera_matrix = None
            self._dist_coeffs = None

        if not self._camera_info_received or valid != self._calibrated:
            if valid:
                self.get_logger().info(
                    f'[Vision] CameraInfo calibration received: fx={k[0]:.1f} fy={k[4]:.1f} '
                    f'cx={k[2]:.1f} cy={k[5]:.1f} -> tag_pose enabled')
            else:
                self.get_logger().warning(
                    '[Vision] CameraInfo is uncalibrated (K[0]==0 or invalid) -> publishing tag_id only, no tag_pose')
        self._camera_info_received = True
        self._calibrated = valid

    # --------------------------------------------------
    # Image processing
    # --------------------------------------------------
    def _on_image(self, msg: Image) -> None:
        self._frames_received += 1
        if self._detector is None:
            return

        try:
            gray = common.imgmsg_to_gray(self._bridge, msg)
        except (CvBridgeError, cv2.error) as exc:
            self._conversion_failures += 1
            self.get_logger().error(f'[Vision] cv_bridge conversion failed: {exc}', throttle_duration_sec=5.0)
            return

        if self._calibrated and self._camera_info_size is not None:
            if (gray.shape[1], gray.shape[0]) != self._camera_info_size:
                self.get_logger().warning(
                    f'[Vision] Image size {gray.shape[1]}x{gray.shape[0]} differs from CameraInfo '
                    f'{self._camera_info_size[0]}x{self._camera_info_size[1]}; pose will be inaccurate',
                    throttle_duration_sec=30.0)

        try:
            detections = self._detector.detect(gray)
        except cv2.error as exc:
            self._detection_failures += 1
            self.get_logger().error(f'[Vision] AprilTag detection failed: {exc}', throttle_duration_sec=5.0)
            return

        selected = self._select_tag(detections)
        rvec = tvec = None

        if selected is not None:
            tag_id, corners = selected
            self._detections += 1
            self._tag_id_pub.publish(String(data=str(tag_id)))

            if self._calibrated:
                pose = self._estimate_pose(corners)
                if pose is not None:
                    rvec, tvec = pose
                    self._tag_pose_pub.publish(self._make_pose_msg(rvec, tvec, msg))
                    self._poses_published += 1
            else:
                self.get_logger().warning(
                    f'[Vision] Tag {tag_id} detected but the camera is not calibrated: tag_pose is not published. '
                    f'Set intrinsics in vision.yaml (camera_node) to enable it.',
                    throttle_duration_sec=self._calibration_warning_period_sec)

        self._log_selection_change(selected)

        if self._debug_pub is not None:
            self._publish_debug_image(gray, detections, selected, rvec, tvec, msg)

    def _select_tag(self, detections: List[Detection]) -> Optional[Detection]:
        """Pick the target id, or the largest tag in view (closest docking marker) when target_tag_id < 0."""
        if not detections:
            return None
        if self._target_tag_id >= 0:
            for det in detections:
                if det[0] == self._target_tag_id:
                    return det
            return None
        return max(detections, key=lambda det: corners_area(det[1]))

    def _log_selection_change(self, selected: Optional[Detection]) -> None:
        """Event log only on change (new tag / tag lost); never per frame."""
        current = None if selected is None else selected[0]
        if current == self._last_selected_id:
            return
        if current is None:
            self.get_logger().info(f'[Vision] Tag {self._last_selected_id} lost')
        else:
            self.get_logger().info(f'[Vision] Tag {current} selected')
        self._last_selected_id = current

    # --------------------------------------------------
    # Pose estimation
    # --------------------------------------------------
    def _estimate_pose(self, corners: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """solvePnP on the four tag corners. Returns (rvec, tvec) or None when it fails a sanity check."""
        image_points = np.ascontiguousarray(corners, dtype=np.float64).reshape(4, 1, 2)
        try:
            ok, rvec, tvec = cv2.solvePnP(self._object_points, image_points, self._camera_matrix,
                                          self._dist_coeffs, flags=self._pnp_flags)
        except cv2.error as exc:
            if self._pnp_flags == cv2.SOLVEPNP_ITERATIVE:
                self.get_logger().error(f'[Vision] solvePnP failed: {exc}', throttle_duration_sec=5.0)
                return None
            # IPPE_SQUARE can reject degenerate corner sets; retry once with the generic solver.
            try:
                ok, rvec, tvec = cv2.solvePnP(self._object_points, image_points, self._camera_matrix,
                                              self._dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
            except cv2.error as exc2:
                self.get_logger().error(f'[Vision] solvePnP failed: {exc2}', throttle_duration_sec=5.0)
                return None
        if not ok:
            self._poses_rejected += 1
            return None

        z = float(np.asarray(tvec).ravel()[2])
        if not math.isfinite(z) or z <= 0.0 or z > self._max_tag_distance_m:
            # A tag behind the camera or absurdly far away is a numerical failure, not a measurement.
            self._poses_rejected += 1
            self.get_logger().warning(f'[Vision] Rejected implausible tag pose (z={z:.2f} m)', throttle_duration_sec=5.0)
            return None
        return rvec, tvec

    def _make_pose_msg(self, rvec: np.ndarray, tvec: np.ndarray, image_msg: Image) -> PoseStamped:
        rot, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = common.rotation_matrix_to_quaternion(rot)
        t = np.asarray(tvec, dtype=np.float64).ravel()

        pose = PoseStamped()
        pose.header.stamp = image_msg.header.stamp
        pose.header.frame_id = image_msg.header.frame_id or DEFAULT_FRAME_ID
        pose.pose.position.x = float(t[0])
        pose.pose.position.y = float(t[1])
        pose.pose.position.z = float(t[2])
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        return pose

    # --------------------------------------------------
    # Debug image (optional, off by default to save CPU on the Pi)
    # --------------------------------------------------
    def _publish_debug_image(self, gray, detections, selected, rvec, tvec, image_msg: Image) -> None:
        debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        selected_id = None if selected is None else selected[0]
        for tag_id, corners in detections:
            colour = (0, 255, 0) if tag_id == selected_id else (0, 165, 255)
            pts = corners.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(debug, [pts], True, colour, 2)
            cv2.putText(debug, str(tag_id), (int(corners[0, 0]), int(corners[0, 1]) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2, cv2.LINE_AA)
        if rvec is not None and tvec is not None and self._camera_matrix is not None:
            axis_len = self._tag_size * 0.5
            axes = np.array([[0.0, 0.0, 0.0], [axis_len, 0.0, 0.0],
                             [0.0, axis_len, 0.0], [0.0, 0.0, axis_len]], dtype=np.float64)
            try:
                proj, _ = cv2.projectPoints(axes, rvec, tvec, self._camera_matrix, self._dist_coeffs)
                proj = proj.reshape(-1, 2).astype(int)
                origin = tuple(proj[0])
                cv2.line(debug, origin, tuple(proj[1]), (0, 0, 255), 2)   # x red
                cv2.line(debug, origin, tuple(proj[2]), (0, 255, 0), 2)   # y green
                cv2.line(debug, origin, tuple(proj[3]), (255, 0, 0), 2)   # z blue
            except cv2.error as exc:
                self.get_logger().warning(f'[Vision] Axis projection failed: {exc}', throttle_duration_sec=10.0)
        try:
            out = self._bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        except (CvBridgeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f'[Vision] Debug image conversion failed: {exc}', throttle_duration_sec=10.0)
            return
        out.header = image_msg.header
        self._debug_pub.publish(out)

    # --------------------------------------------------
    # Heartbeat
    # --------------------------------------------------
    def _on_heartbeat(self) -> None:
        self.get_logger().info(
            f'[Vision] AprilTag alive | enabled={self._enabled} | frames_received={self._frames_received} | '
            f'detections={self._detections} | poses_published={self._poses_published} | '
            f'poses_rejected={self._poses_rejected} | camera_calibrated={self._calibrated} | '
            f'camera_info_received={self._camera_info_received} | '
            f'conversion_failures={self._conversion_failures} | detection_failures={self._detection_failures}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AprilTagNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:  # noqa: BLE001 - e.g. a parameter type error in vision.yaml
        get_logger('apriltag_node').fatal(f'[Vision] apriltag_node failed to start or crashed: {exc!r}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
