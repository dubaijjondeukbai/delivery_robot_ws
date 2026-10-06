#!/usr/bin/env python3
"""
[Vision] camera_node

Role
----
Manage the camera input and publish images + camera intrinsics.

    camera (Raspberry Pi Camera Module 3 Wide | USB UVC | mock)
            |
            v
       camera_node  --->  /camera/image_raw   (sensor_msgs/Image, bgr8)
                    --->  /camera/camera_info (sensor_msgs/CameraInfo, same stamp)

Hardware
--------
Front camera: Raspberry Pi Camera Module 3 Wide (Sony IMX708, CSI, 102 deg horizontal
FOV, autofocus). A CSI camera is NOT a UVC device: on the Pi it is served by libcamera,
so plain cv2.VideoCapture(/dev/videoN) does not deliver frames. This node therefore has
several frame sources (parameter camera_backend):

    picamera2  : libcamera through the picamera2 Python library (best on the Pi)
    gstreamer  : cv2.VideoCapture("libcamerasrc ! ... ! appsink", CAP_GSTREAMER)
                 (needs gstreamer1.0-libcamera and an OpenCV built with GStreamer, as Ubuntu's is)
    v4l2       : cv2.VideoCapture(camera_id) - USB UVC webcams (the previous default)
    auto       : try picamera2 -> gstreamer -> v4l2 and keep the first that delivers a frame

Modes
-----
use_camera = false  (default)
    MOCK MODE. No hardware access. A black bgr8 frame of the configured size is
    published at publish_rate_hz so that apriltag_node / qr_node / detector_node
    can be tested without a real camera.

use_camera = true
    REAL MODE. One of the frame sources above. A failed open or a dropped frame
    never kills the node: it keeps counting failures and reopens the source
    periodically.

CameraInfo
----------
Follows the sensor_msgs convention: for a camera that has not been calibrated yet
(intrinsics.fx/fy == 0) the K/D/R/P matrices are published ZEROED (K[0] == 0 marks
"uncalibrated") and the node says so in the log. apriltag_node then publishes
tag_id only (no tag_pose) until real intrinsics are entered in vision.yaml.

Safety / architecture
---------------------
This node is a pure camera driver. It performs no detection of any kind, so the
perception nodes depend only on the Image/CameraInfo topics. In simulation the
node is not started at all: the Gazebo camera publishes the same topics.
"""

from __future__ import annotations

import math
import sys
import time
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from robot_vision import common

# ==================================================
# TOPIC NAMES
# Defaults live in robot_vision/common.py (single source for every vision node)
# and can be overridden in vision.yaml (image_topic / camera_info_topic).
# Do NOT hard-code topic strings anywhere else in this file.
# ==================================================
DEFAULT_IMAGE_TOPIC = common.TOPIC_IMAGE_RAW
DEFAULT_CAMERA_INFO_TOPIC = common.TOPIC_CAMERA_INFO
DEFAULT_FRAME_ID = common.DEFAULT_CAMERA_FRAME_ID

MODE_MOCK = 'mock'
MODE_REAL = 'real'
DISTORTION_MODEL_PLUMB_BOB = 'plumb_bob'
IMAGE_ENCODING = 'bgr8'

BACKEND_AUTO = 'auto'
BACKEND_PICAMERA2 = 'picamera2'
BACKEND_GSTREAMER = 'gstreamer'
BACKEND_V4L2 = 'v4l2'
BACKEND_ORDER_AUTO = (BACKEND_PICAMERA2, BACKEND_GSTREAMER, BACKEND_V4L2)
BACKENDS = (BACKEND_AUTO,) + BACKEND_ORDER_AUTO


# ==================================================
# FRAME SOURCES
# One small class per way of getting a BGR frame. The node only talks to the
# FrameSource interface, so adding a camera later means adding a class here.
# ==================================================
class FrameSource:
    """Minimal interface: open() -> bool, read() -> BGR uint8 array | None, release()."""

    name = 'abstract'

    def open(self) -> bool:
        raise NotImplementedError

    def read(self) -> Optional[np.ndarray]:
        raise NotImplementedError

    def release(self) -> None:
        raise NotImplementedError

    def describe(self) -> str:
        return self.name


class OpenCvSource(FrameSource):
    """cv2.VideoCapture on a V4L2 device index (USB UVC webcams)."""

    name = BACKEND_V4L2

    def __init__(self, camera_id: int, width: int, height: int, fps: float, fourcc: str, logger) -> None:
        self._camera_id = camera_id
        self._width = width
        self._height = height
        self._fps = fps
        self._fourcc = fourcc
        self._log = logger
        self._cap = None  # type: Optional[cv2.VideoCapture]
        self._actual = ''

    def open(self) -> bool:
        self.release()
        backends = []
        if sys.platform.startswith('linux') and hasattr(cv2, 'CAP_V4L2'):
            backends.append(('V4L2', cv2.CAP_V4L2))
        backends.append(('default', cv2.CAP_ANY))
        cap = None
        for label, backend in backends:
            try:
                candidate = cv2.VideoCapture(self._camera_id, backend)
            except cv2.error as exc:
                self._log.warning(f'[Vision] VideoCapture with {label} backend raised: {exc}')
                continue
            if candidate is not None and candidate.isOpened():
                cap = candidate
                break
            if candidate is not None:
                candidate.release()
        if cap is None:
            return False
        # Requests only; not every camera honours them, so the actual values are read back.
        if self._fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self._fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self._width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self._height))
        cap.set(cv2.CAP_PROP_FPS, float(self._fps))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1.0)   # keep the driver queue short (ignored if unsupported)
        self._actual = (f'{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}'
                        f' @ {cap.get(cv2.CAP_PROP_FPS):.1f} fps')
        self._cap = cap
        return True

    def read(self) -> Optional[np.ndarray]:
        if self._cap is None or not self._cap.isOpened():
            return None
        ok, frame = self._cap.read()
        if not ok or frame is None or frame.size == 0:
            return None
        return frame

    def release(self) -> None:
        if self._cap is not None:
            try:
                self._cap.release()
            except cv2.error as exc:
                self._log.warning(f'[Vision] Error while releasing camera: {exc}')
            self._cap = None

    def describe(self) -> str:
        return f'v4l2 camera_id={self._camera_id} ({self._actual})'


class GStreamerSource(FrameSource):
    """cv2.VideoCapture on a GStreamer pipeline (libcamerasrc for the CSI camera).

    Requires an OpenCV built with GStreamer (Ubuntu's python3-opencv is) and the
    gstreamer1.0-libcamera plugin on the Pi. The default pipeline lets libcamerasrc pick
    its native format and converts to BGR, which is what the rest of the node expects.
    """

    name = BACKEND_GSTREAMER

    def __init__(self, pipeline: str, width: int, height: int, fps: float, logger) -> None:
        self._pipeline = pipeline.strip() or self.default_pipeline(width, height, fps)
        self._log = logger
        self._cap = None  # type: Optional[cv2.VideoCapture]

    @staticmethod
    def default_pipeline(width: int, height: int, fps: float) -> str:
        return (f'libcamerasrc ! video/x-raw,width={width},height={height},framerate={int(round(fps))}/1 '
                f'! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false')

    @staticmethod
    def opencv_has_gstreamer() -> bool:
        """True when cv2.getBuildInformation() reports 'GStreamer: YES (...)'."""
        try:
            for line in cv2.getBuildInformation().splitlines():
                if line.strip().startswith('GStreamer:'):
                    return 'YES' in line
        except cv2.error:
            pass
        return False

    def open(self) -> bool:
        self.release()
        if not hasattr(cv2, 'CAP_GSTREAMER') or not self.opencv_has_gstreamer():
            self._log.warning('[Vision] OpenCV was built without GStreamer; gstreamer backend unavailable')
            return False
        try:
            cap = cv2.VideoCapture(self._pipeline, cv2.CAP_GSTREAMER)
        except cv2.error as exc:
            self._log.warning(f'[Vision] GStreamer pipeline raised: {exc}')
            return False
        if cap is None or not cap.isOpened():
            if cap is not None:
                cap.release()
            return False
        self._cap = cap
        return True

    def read(self) -> Optional[np.ndarray]:
        if self._cap is None or not self._cap.isOpened():
            return None
        ok, frame = self._cap.read()
        if not ok or frame is None or frame.size == 0:
            return None
        return frame

    def release(self) -> None:
        if self._cap is not None:
            try:
                self._cap.release()
            except cv2.error as exc:
                self._log.warning(f'[Vision] Error while releasing GStreamer pipeline: {exc}')
            self._cap = None

    def describe(self) -> str:
        return f'gstreamer "{self._pipeline}"'


class Picamera2Source(FrameSource):
    """libcamera through picamera2 (Raspberry Pi Camera Module 3 and other CSI cameras).

    picamera2's "RGB888" delivers numpy arrays in B,G,R byte order, i.e. exactly the bgr8
    layout OpenCV and this node use. If the colours come out swapped on your install,
    set picamera2_format to "BGR888".
    """

    name = BACKEND_PICAMERA2

    def __init__(self, camera_num: int, width: int, height: int, fps: float, pixel_format: str,
                 autofocus: bool, logger) -> None:
        self._camera_num = camera_num
        self._width = width
        self._height = height
        self._fps = fps
        self._format = pixel_format or 'RGB888'
        self._autofocus = autofocus
        self._log = logger
        self._cam = None
        self._info = ''

    def open(self) -> bool:
        self.release()
        try:
            from picamera2 import Picamera2  # optional dependency, only present on the Pi
        except ImportError:
            self._log.info('[Vision] picamera2 not installed; skipping that backend')
            return False
        try:
            cam = Picamera2(self._camera_num)
            config = cam.create_video_configuration(
                main={'size': (self._width, self._height), 'format': self._format},
                controls={'FrameRate': float(self._fps)},
            )
            cam.configure(config)
            if self._autofocus:
                try:
                    from libcamera import controls as lc_controls
                    cam.set_controls({'AfMode': lc_controls.AfModeEnum.Continuous})
                except Exception as exc:  # noqa: BLE001 - fixed-focus modules have no AF controls
                    self._log.info(f'[Vision] Autofocus not available on this camera: {exc}')
            cam.start()
            props = getattr(cam, 'camera_properties', {}) or {}
            self._info = f"{props.get('Model', 'unknown model')} {self._width}x{self._height} @ {self._fps:.0f} fps {self._format}"
        except Exception as exc:  # noqa: BLE001 - libcamera raises many exception types
            self._log.warning(f'[Vision] picamera2 open failed: {exc!r}')
            self.release()
            return False
        self._cam = cam
        return True

    def read(self) -> Optional[np.ndarray]:
        if self._cam is None:
            return None
        try:
            frame = self._cam.capture_array('main')
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f'[Vision] picamera2 capture failed: {exc!r}', throttle_duration_sec=5.0)
            return None
        if frame is None or frame.size == 0:
            return None
        if frame.ndim == 3 and frame.shape[2] == 4:      # XRGB/XBGR -> drop the padding channel
            frame = np.ascontiguousarray(frame[:, :, :3])
        return frame

    def release(self) -> None:
        if self._cam is not None:
            try:
                self._cam.stop()
                self._cam.close()
            except Exception as exc:  # noqa: BLE001
                self._log.warning(f'[Vision] Error while closing picamera2: {exc!r}')
            self._cam = None

    def describe(self) -> str:
        return f'picamera2 camera_num={self._camera_num} ({self._info})'


class CameraNode(Node):
    """Camera driver node (CSI / USB UVC / mock)."""

    def __init__(self) -> None:
        super().__init__('camera_node')

        self._bridge = CvBridge()
        self._source = None  # type: Optional[FrameSource]
        self._mock_frame = None  # type: Optional[np.ndarray]

        # Heartbeat statistics.
        self._frames_published = 0
        self._frames_at_last_heartbeat = 0
        self._capture_failures = 0
        self._consecutive_failures = 0
        self._last_reopen_attempt = 0.0
        self._size_warning_done = False

        self._declare_parameters()
        self._load_parameters()

        self._camera_info_template = self._build_camera_info_template()

        self._image_pub = self.create_publisher(Image, self._image_topic, qos_profile_sensor_data)
        self._info_pub = self.create_publisher(CameraInfo, self._camera_info_topic,
                                               qos_profile_sensor_data)

        if self._mode == MODE_MOCK:
            self._mock_frame = np.zeros((self._image_height, self._image_width, 3), dtype=np.uint8)
            self.get_logger().info(
                f'[Vision] Mock camera mode: publishing black {self._image_width}x{self._image_height} '
                f'{IMAGE_ENCODING} frames at {self._publish_rate_hz:.1f} Hz (no hardware access)')
        else:
            if not self._open_camera():
                self.get_logger().error(
                    f'[Vision] Could not open any camera (backend={self._backend}). The node stays alive '
                    f'and retries every {self._reopen_interval_sec:.1f} s. On the Pi check `rpicam-hello` / '
                    f'`cam -l` (libcamera), for USB cameras `ls /dev/video*`; or run with use_camera:=false '
                    f'for mock mode.')

        self._capture_timer = self.create_timer(1.0 / self._publish_rate_hz, self._on_capture_timer)
        self._heartbeat_timer = self.create_timer(self._heartbeat_period_sec, self._on_heartbeat)

        self.get_logger().info(
            f'[Vision] camera_node ready: mode={self._mode}, backend={self._backend}, '
            f'image_topic={self._image_topic}, camera_info_topic={self._camera_info_topic}, '
            f'frame_id={self._frame_id}')

    # --------------------------------------------------
    # Parameters
    # --------------------------------------------------
    def _declare_parameters(self) -> None:
        self.declare_parameter('use_camera', False)
        # Frame source: auto | picamera2 | gstreamer | v4l2 (see module docstring)
        self.declare_parameter('camera_backend', BACKEND_AUTO)
        self.declare_parameter('camera_id', 0)              # v4l2 device index / picamera2 camera number
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('frame_id', DEFAULT_FRAME_ID)
        # FourCC pixel format request for v4l2, e.g. "MJPG". Empty string = leave the driver default.
        self.declare_parameter('fourcc', '')
        # GStreamer pipeline; empty = libcamerasrc pipeline built from width/height/rate.
        self.declare_parameter('gstreamer_pipeline', '')
        # picamera2 options
        self.declare_parameter('picamera2_format', 'RGB888')   # B,G,R byte order = bgr8
        self.declare_parameter('picamera2_autofocus', True)    # Camera Module 3 has PDAF autofocus
        self.declare_parameter('image_topic', DEFAULT_IMAGE_TOPIC)
        self.declare_parameter('camera_info_topic', DEFAULT_CAMERA_INFO_TOPIC)
        # Intrinsics (pixels). 0.0 means "not calibrated yet".
        self.declare_parameter('intrinsics.fx', 0.0)
        self.declare_parameter('intrinsics.fy', 0.0)
        self.declare_parameter('intrinsics.cx', 0.0)
        self.declare_parameter('intrinsics.cy', 0.0)
        # plumb_bob distortion: k1 k2 p1 p2 k3.
        self.declare_parameter('distortion.k1', 0.0)
        self.declare_parameter('distortion.k2', 0.0)
        self.declare_parameter('distortion.p1', 0.0)
        self.declare_parameter('distortion.p2', 0.0)
        self.declare_parameter('distortion.k3', 0.0)
        # Recovery behaviour.
        self.declare_parameter('reopen_after_failures', 15)
        self.declare_parameter('reopen_interval_sec', 2.0)
        self.declare_parameter('heartbeat_period_sec', 5.0)

    def _load_parameters(self) -> None:
        log = self.get_logger()
        p = self.get_parameter

        self._mode = MODE_REAL if bool(p('use_camera').value) else MODE_MOCK
        self._backend = str(p('camera_backend').value).strip().lower() or BACKEND_AUTO
        if self._backend not in BACKENDS:
            log.error(f"[Vision] Parameter 'camera_backend'={self._backend!r} must be one of {BACKENDS}; using 'auto'")
            self._backend = BACKEND_AUTO
        self._camera_id = int(p('camera_id').value)
        self._image_width = int(common.positive_or_default(log, 'image_width', p('image_width').value, 640))
        self._image_height = int(common.positive_or_default(log, 'image_height', p('image_height').value, 480))
        self._publish_rate_hz = float(common.positive_or_default(log, 'publish_rate_hz',
                                                                 p('publish_rate_hz').value, 15.0))
        self._frame_id = str(p('frame_id').value) or DEFAULT_FRAME_ID
        self._fourcc = str(p('fourcc').value).strip()
        if self._fourcc and len(self._fourcc) != 4:
            log.error(f"[Vision] Parameter 'fourcc'={self._fourcc!r} must be exactly 4 characters; ignoring it")
            self._fourcc = ''
        self._gstreamer_pipeline = str(p('gstreamer_pipeline').value)
        self._picamera2_format = str(p('picamera2_format').value).strip() or 'RGB888'
        self._picamera2_autofocus = bool(p('picamera2_autofocus').value)
        self._image_topic = str(p('image_topic').value) or DEFAULT_IMAGE_TOPIC
        self._camera_info_topic = str(p('camera_info_topic').value) or DEFAULT_CAMERA_INFO_TOPIC

        self._fx = float(p('intrinsics.fx').value)
        self._fy = float(p('intrinsics.fy').value)
        self._cx = float(p('intrinsics.cx').value)
        self._cy = float(p('intrinsics.cy').value)
        self._dist = [float(p(f'distortion.{k}').value) for k in ('k1', 'k2', 'p1', 'p2', 'k3')]

        self._reopen_after_failures = max(1, int(p('reopen_after_failures').value))
        self._reopen_interval_sec = float(common.positive_or_default(log, 'reopen_interval_sec',
                                                                     p('reopen_interval_sec').value, 2.0))
        self._heartbeat_period_sec = float(common.positive_or_default(log, 'heartbeat_period_sec',
                                                                      p('heartbeat_period_sec').value, 5.0))

        # Calibration state: fx and fy must both be positive and finite.
        self._calibrated = all(math.isfinite(v) for v in (self._fx, self._fy, self._cx, self._cy)) \
            and self._fx > 0.0 and self._fy > 0.0
        if self._calibrated:
            if self._cx <= 0.0 or self._cy <= 0.0:
                log.warning('[Vision] intrinsics.fx/fy are set but cx/cy are <= 0; the principal point is '
                            'probably wrong (expected roughly image centre)')
            log.info(f'[Vision] Camera intrinsics: fx={self._fx:.2f} fy={self._fy:.2f} '
                     f'cx={self._cx:.2f} cy={self._cy:.2f} dist={self._dist}')
        else:
            log.warning('[Vision] Camera is NOT calibrated (intrinsics.fx/fy are 0). CameraInfo is published '
                        'with zeroed K/D/R/P (K[0]==0). AprilTag pose estimation will be disabled '
                        'until real values are set in vision.yaml. TODO: calibrate the camera.')

    # --------------------------------------------------
    # CameraInfo
    # --------------------------------------------------
    def _build_camera_info_template(self) -> CameraInfo:
        """Build the CameraInfo once; only header/size are updated per frame."""
        info = CameraInfo()
        info.width = self._image_width
        info.height = self._image_height
        info.distortion_model = DISTORTION_MODEL_PLUMB_BOB
        if self._calibrated:
            fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy
            info.d = [float(v) for v in self._dist]
            info.k = [fx, 0.0, cx,
                      0.0, fy, cy,
                      0.0, 0.0, 1.0]
            # Monocular camera: R is identity and P is K extended with a zero translation column.
            info.r = [1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0]
            info.p = [fx, 0.0, cx, 0.0,
                      0.0, fy, cy, 0.0,
                      0.0, 0.0, 1.0, 0.0]
        else:
            # sensor_msgs/CameraInfo: an uncalibrated camera leaves D, K, R, P zeroed.
            info.d = [0.0] * 5
            info.k = [0.0] * 9
            info.r = [0.0] * 9
            info.p = [0.0] * 12
        return info

    # --------------------------------------------------
    # Camera handling
    # --------------------------------------------------
    def _make_source(self, backend: str) -> FrameSource:
        log = self.get_logger()
        if backend == BACKEND_PICAMERA2:
            return Picamera2Source(self._camera_id, self._image_width, self._image_height,
                                   self._publish_rate_hz, self._picamera2_format,
                                   self._picamera2_autofocus, log)
        if backend == BACKEND_GSTREAMER:
            return GStreamerSource(self._gstreamer_pipeline, self._image_width, self._image_height,
                                   self._publish_rate_hz, log)
        return OpenCvSource(self._camera_id, self._image_width, self._image_height,
                            self._publish_rate_hz, self._fourcc, log)

    def _candidate_backends(self) -> List[str]:
        return list(BACKEND_ORDER_AUTO) if self._backend == BACKEND_AUTO else [self._backend]

    def _open_camera(self) -> bool:
        """Open the first backend that both opens AND delivers a frame (a CSI camera can 'open' via V4L2 and never deliver)."""
        self._release_camera()
        self._last_reopen_attempt = time.monotonic()

        for backend in self._candidate_backends():
            source = self._make_source(backend)
            if not source.open():
                self.get_logger().info(f'[Vision] Backend {backend}: not available')
                continue
            frame = None
            for _ in range(5):                  # a few tries: libcamera needs a moment for the first frame
                frame = source.read()
                if frame is not None:
                    break
                time.sleep(0.1)
            if frame is None:
                self.get_logger().warning(f'[Vision] Backend {backend} opened but delivered no frame; trying the next one')
                source.release()
                continue
            self._source = source
            self._consecutive_failures = 0
            self.get_logger().info(f'[Vision] Camera opened: {source.describe()}')
            self._check_frame_size(frame)
            return True
        return False

    def _check_frame_size(self, frame: np.ndarray) -> None:
        actual_h, actual_w = frame.shape[:2]
        if (actual_w, actual_h) != (self._image_width, self._image_height) and not self._size_warning_done:
            self._size_warning_done = True
            self.get_logger().warning(
                f'[Vision] Camera delivers {actual_w}x{actual_h} instead of the requested '
                f'{self._image_width}x{self._image_height}. CameraInfo width/height follow the real '
                f'frame, but intrinsics (fx, fy, cx, cy) were calibrated for the requested size '
                f'and will be wrong if the resolution differs.')

    def _release_camera(self) -> None:
        if self._source is not None:
            self._source.release()
            self._source = None

    def _maybe_reopen(self) -> None:
        """Reopen the device after repeated failures, rate limited by reopen_interval_sec."""
        if self._consecutive_failures < self._reopen_after_failures:
            return
        now = time.monotonic()
        if now - self._last_reopen_attempt < self._reopen_interval_sec:
            return
        self.get_logger().warning(
            f'[Vision] {self._consecutive_failures} consecutive capture failures; reopening the camera')
        if self._open_camera():
            self.get_logger().info('[Vision] Camera reopened successfully')
        else:
            self.get_logger().error('[Vision] Camera reopen failed; will retry', throttle_duration_sec=10.0)

    def _read_frame(self) -> Optional[np.ndarray]:
        """Grab one frame from the source. Returns None (and counts the failure) on error."""
        if self._source is None:
            self._capture_failures += 1
            self._consecutive_failures += 1
            self._maybe_reopen()
            return None
        try:
            frame = self._source.read()
        except cv2.error as exc:
            frame = None
            self.get_logger().warning(f'[Vision] Camera read raised: {exc}', throttle_duration_sec=5.0)
        if frame is None:
            self._capture_failures += 1
            self._consecutive_failures += 1
            self.get_logger().warning('[Vision] Camera frame read failed', throttle_duration_sec=5.0)
            self._maybe_reopen()
            return None
        self._consecutive_failures = 0
        return frame

    # --------------------------------------------------
    # Timers
    # --------------------------------------------------
    def _on_capture_timer(self) -> None:
        try:
            frame = self._mock_frame if self._mode == MODE_MOCK else self._read_frame()
            if frame is None:
                return
            self._publish_frame(frame)
        except Exception as exc:  # noqa: BLE001 - last line of defence: a driver hiccup must not kill the node
            self._capture_failures += 1
            self.get_logger().error(f'[Vision] Unexpected error in capture loop: {exc!r}', throttle_duration_sec=5.0)

    def _publish_frame(self, frame: np.ndarray) -> None:
        """Publish the frame and a CameraInfo carrying the identical timestamp."""
        stamp = self.get_clock().now().to_msg()
        try:
            img_msg = self._bridge.cv2_to_imgmsg(frame, encoding=IMAGE_ENCODING)
        except (CvBridgeError, TypeError, ValueError) as exc:
            self._capture_failures += 1
            self.get_logger().error(f'[Vision] cv_bridge conversion failed: {exc}', throttle_duration_sec=5.0)
            return
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self._frame_id

        info = self._camera_info_template
        info.header.stamp = stamp
        info.header.frame_id = self._frame_id
        info.height = int(frame.shape[0])
        info.width = int(frame.shape[1])

        self._image_pub.publish(img_msg)
        self._info_pub.publish(info)
        self._frames_published += 1

    def _on_heartbeat(self) -> None:
        published_since_last = self._frames_published - self._frames_at_last_heartbeat
        self._frames_at_last_heartbeat = self._frames_published
        fps = published_since_last / self._heartbeat_period_sec
        camera_open = self._mode == MODE_MOCK or self._source is not None
        backend = 'mock' if self._mode == MODE_MOCK else (self._source.name if self._source else 'none')
        self.get_logger().info(
            f'[Vision] camera alive | mode={self._mode} | backend={backend} | camera_open={camera_open} | '
            f'frames_published={self._frames_published} | capture_failures={self._capture_failures} | '
            f'fps={fps:.1f} | calibrated={self._calibrated}')

    # --------------------------------------------------
    # Shutdown
    # --------------------------------------------------
    def destroy_node(self):
        self._release_camera()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:  # noqa: BLE001 - e.g. a parameter type error in vision.yaml
        get_logger('camera_node').fatal(f'[Vision] camera_node failed to start or crashed: {exc!r}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
