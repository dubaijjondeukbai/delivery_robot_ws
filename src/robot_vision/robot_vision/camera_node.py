#!/usr/bin/env python3
"""
Camera Module 3 / USB / mock 영상과 CameraInfo를 발행한다.

인스턴스 하나당 카메라 한 대를 담당한다.
두 대는 노드를 두 번 실행하고 다음을 각각 다르게 지정한다.

- 노드 이름
- camera_id 또는 camera_unique_id
- image_topic
- camera_info_topic
- frame_id
- 카메라 보정값

카메라 위치와 방향(TF)은 URDF에서 정의한다.

출력
----
Image:
    bgr8 영상.

CameraInfo:
    영상과 동일한 stamp, frame_id, 실제 해상도.
    보정 조건을 만족하지 않으면 K/D/R/P를 0으로 발행.

시간
----
stamp는 호스트에서 프레임을 받은 직후의 ROS 시각이다.
센서의 실제 노출 시각이나 두 카메라의 동기화를 의미하지 않는다.

설정
----
파라미터는 시작 시 읽는다.
설정을 변경한 뒤에는 노드를 재시작한다.
"""

import math
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from robot_vision import common


# ============================================================
# 영상 형식 검증
# ============================================================
def bgr_frame(frame):
    """
    비어 있지 않은 H x W x 3, uint8 BGR 영상만 허용한다.

    임의로 resize하거나 잘못된 채널 수를 잘라내지 않는다.
    """
    if (
        not isinstance(frame, np.ndarray)
        or frame.dtype != np.uint8
        or frame.ndim != 3
        or frame.shape[2] != 3
        or frame.shape[0] < 1
        or frame.shape[1] < 1
    ):
        raise ValueError(
            'Expected a non-empty HxWx3 uint8 BGR frame'
        )

    return np.ascontiguousarray(frame)


# ============================================================
# 카메라 입력
# ============================================================
class CameraSource:
    """카메라 핸들을 열고, 읽고, 해제한다."""

    def __init__(self, backend, config, logger):
        self.backend = backend
        self.config = config
        self.log = logger

        self._cam = None
        self._cap = None

    def open(self):
        c = self.config

        try:
            # ------------------------------------------------
            # Raspberry Pi CSI 카메라
            # ------------------------------------------------
            if self.backend == 'picamera2':
                from picamera2 import Picamera2

                camera_num = c['camera_id']
                unique_id = c['camera_unique_id']

                # 고유 ID가 지정됐다면 번호보다 우선한다.
                if unique_id:
                    matches = [
                        item
                        for item in Picamera2.global_camera_info()
                        if item.get('Id') == unique_id
                    ]

                    if len(matches) != 1:
                        raise RuntimeError(
                            'camera_unique_id not uniquely found'
                        )

                    camera_num = int(matches[0]['Num'])

                # configure/start 도중 실패해도 해제할 수 있도록
                # 객체를 생성하자마자 저장한다.
                self._cam = Picamera2(camera_num)

                video = self._cam.create_video_configuration(
                    main={
                        'size': (
                            c['image_width'],
                            c['image_height'],
                        ),
                        'format': c['picamera2_format'],
                    },
                    controls={
                        'FrameRate': c['publish_rate_hz'],
                    },
                    buffer_count=4,
                    queue=False,
                )

                self._cam.configure(video)

                if 'AfMode' in self._cam.camera_controls:
                    from libcamera import controls

                    if c['picamera2_autofocus']:
                        focus = {
                            'AfMode':
                                controls.AfModeEnum.Continuous,
                        }
                    else:
                        focus = {
                            'AfMode':
                                controls.AfModeEnum.Manual,
                            'LensPosition':
                                c['picamera2_lens_position'],
                        }

                    self._cam.set_controls(focus)

                self._cam.start()

                self.log.info(
                    f'[Camera] Picamera2 camera_num={camera_num}'
                )

            # ------------------------------------------------
            # 명시적으로 지정한 GStreamer pipeline
            # ------------------------------------------------
            elif self.backend == 'gstreamer':
                pipeline = c['gstreamer_pipeline']

                # 두 카메라가 같은 장치를 자동 선택하지 않도록
                # 사용할 카메라를 지정한 pipeline을 요구한다.
                if not pipeline:
                    raise RuntimeError(
                        'gstreamer_pipeline must select '
                        'the camera explicitly'
                    )

                available = any(
                    line.strip().startswith('GStreamer:')
                    and 'YES' in line
                    for line in (
                        cv2.getBuildInformation().splitlines()
                    )
                )

                if not available:
                    raise RuntimeError(
                        'OpenCV has no GStreamer support'
                    )

                self._cap = cv2.VideoCapture(
                    pipeline,
                    cv2.CAP_GSTREAMER,
                )

                if not self._cap.isOpened():
                    raise RuntimeError(
                        'Cannot open GStreamer pipeline'
                    )

            # ------------------------------------------------
            # USB UVC 카메라
            # ------------------------------------------------
            elif self.backend == 'v4l2':
                api = (
                    cv2.CAP_V4L2
                    if sys.platform.startswith('linux')
                    else cv2.CAP_ANY
                )

                self._cap = cv2.VideoCapture(
                    c['camera_id'],
                    api,
                )

                if not self._cap.isOpened():
                    raise RuntimeError(
                        'Cannot open USB camera'
                    )

                if c['fourcc']:
                    self._cap.set(
                        cv2.CAP_PROP_FOURCC,
                        cv2.VideoWriter_fourcc(*c['fourcc']),
                    )

                self._cap.set(
                    cv2.CAP_PROP_FRAME_WIDTH,
                    c['image_width'],
                )
                self._cap.set(
                    cv2.CAP_PROP_FRAME_HEIGHT,
                    c['image_height'],
                )
                self._cap.set(
                    cv2.CAP_PROP_FPS,
                    c['publish_rate_hz'],
                )
                self._cap.set(
                    cv2.CAP_PROP_BUFFERSIZE,
                    1,
                )

            else:
                raise ValueError(
                    f'Unknown backend: {self.backend}'
                )

            return True

        except Exception as exc:
            self.close()

            self.log.warning(
                f'[Camera] {self.backend} open failed: {exc!r}'
            )

            return False

    def read(self):
        if self._cam is not None:
            frame = self._cam.capture_array('main')

            # Picamera2의 RGB888 배열은 BGR 순서다.
            # BGR888 배열은 RGB이므로 BGR로 변환한다.
            if (
                frame is not None
                and self.config['picamera2_format'] == 'BGR888'
            ):
                frame = cv2.cvtColor(
                    frame,
                    cv2.COLOR_RGB2BGR,
                )

            return frame

        if self._cap is not None:
            ok, frame = self._cap.read()
            return frame if ok else None

        return None

    def close(self):
        cam, cap = self._cam, self._cap

        self._cam = None
        self._cap = None

        if cam is not None:
            # stop 실패와 무관하게 close를 시도한다.
            try:
                cam.stop()
            except Exception:
                pass

            try:
                cam.close()
            except Exception as exc:
                self.log.warning(
                    f'[Camera] Picamera2 close failed: {exc!r}'
                )

        if cap is not None:
            try:
                cap.release()
            except Exception as exc:
                self.log.warning(
                    f'[Camera] OpenCV release failed: {exc!r}'
                )


# ============================================================
# ROS 노드
# ============================================================
class CameraNode(Node):
    def __init__(self):
        # launch 또는 --ros-args의 __node remapping으로
        # 카메라별 노드 이름을 지정할 수 있다.
        super().__init__('camera_node')

        defaults = {
            'use_camera': False,
            'camera_backend': 'auto',

            'camera_id': 0,
            'camera_unique_id': '',

            'image_width': 640,
            'image_height': 480,
            'publish_rate_hz': 15.0,

            'frame_id': common.DEFAULT_CAMERA_FRAME_ID,
            'image_topic': common.TOPIC_IMAGE_RAW,
            'camera_info_topic': common.TOPIC_CAMERA_INFO,

            'fourcc': '',
            'gstreamer_pipeline': '',

            'picamera2_format': 'RGB888',
            'picamera2_autofocus': True,
            'picamera2_lens_position': 0.0,

            # 실제 보정에 사용한 영상 해상도.
            # 0이면 보정 해상도가 등록되지 않은 상태다.
            'calibration_width': 0,
            'calibration_height': 0,

            'intrinsics.fx': 0.0,
            'intrinsics.fy': 0.0,
            'intrinsics.cx': 0.0,
            'intrinsics.cy': 0.0,

            'distortion.k1': 0.0,
            'distortion.k2': 0.0,
            'distortion.p1': 0.0,
            'distortion.p2': 0.0,
            'distortion.k3': 0.0,

            'reopen_after_failures': 15,
            'reopen_interval_sec': 2.0,
            'heartbeat_period_sec': 5.0,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.c = {
            name: self.get_parameter(name).value
            for name in defaults
        }

        self._validate_config()

        self._bridge = CvBridge()
        self._source = None

        self._next_open = 0.0
        self._consecutive_failures = 0
        self._capture_failures = 0

        self._published = 0
        self._last_published = None
        self._heartbeat_count = 0
        self._heartbeat_time = time.monotonic()

        self._info_reason = None
        self._warned_size = None

        if not self.c['use_camera']:
            self._mock = np.zeros(
                (
                    self.c['image_height'],
                    self.c['image_width'],
                    3,
                ),
                dtype=np.uint8,
            )
        else:
            self._mock = None

        self._image_pub = self.create_publisher(
            Image,
            self.c['image_topic'],
            qos_profile_sensor_data,
        )

        self._info_pub = self.create_publisher(
            CameraInfo,
            self.c['camera_info_topic'],
            qos_profile_sensor_data,
        )

        self._capture_timer = self.create_timer(
            1.0 / self.c['publish_rate_hz'],
            self._on_capture,
        )

        self._heartbeat_timer = self.create_timer(
            self.c['heartbeat_period_sec'],
            self._on_heartbeat,
        )

        self.get_logger().info(
            f"[Camera] ready: real={self.c['use_camera']} "
            f"id={self.c['camera_id']} "
            f"backend={self.c['camera_backend']} "
            f"image={self.c['image_topic']} "
            f"frame={self.c['frame_id']}"
        )

    # ======================================================
    # 설정 검사
    # ======================================================
    def _validate_config(self):
        for key, value in self.c.items():
            if key in ('use_camera', 'picamera2_autofocus'):
                if type(value) is not bool:
                    raise ValueError(
                        f'{key} must be bool'
                    )

            elif isinstance(value, (int, float)):
                if (
                    isinstance(value, bool)
                    or not math.isfinite(value)
                ):
                    raise ValueError(
                        f'{key} must be finite'
                    )

        positive_ints = (
            'image_width',
            'image_height',
            'reopen_after_failures',
        )

        nonnegative_ints = (
            'camera_id',
            'calibration_width',
            'calibration_height',
        )

        for key in positive_ints + nonnegative_ints:
            value = self.c[key]
            lower = 1 if key in positive_ints else 0

            if type(value) is not int or value < lower:
                raise ValueError(
                    f'Invalid integer parameter: {key}'
                )

        for key in (
            'publish_rate_hz',
            'reopen_interval_sec',
            'heartbeat_period_sec',
        ):
            if self.c[key] <= 0:
                raise ValueError(
                    f'{key} must be > 0'
                )

        for key in (
            'picamera2_lens_position',
            'intrinsics.fx',
            'intrinsics.fy',
        ):
            if self.c[key] < 0:
                raise ValueError(
                    f'{key} must be >= 0'
                )

        for key in (
            'frame_id',
            'image_topic',
            'camera_info_topic',
        ):
            if (
                not isinstance(self.c[key], str)
                or not self.c[key].strip()
            ):
                raise ValueError(
                    f'{key} must be non-empty text'
                )

            self.c[key] = self.c[key].strip()

        for key in (
            'camera_backend',
            'camera_unique_id',
            'fourcc',
            'gstreamer_pipeline',
            'picamera2_format',
        ):
            if not isinstance(self.c[key], str):
                raise ValueError(
                    f'{key} must be text'
                )

            self.c[key] = self.c[key].strip()

        if self.c['camera_backend'] not in (
            'auto',
            'picamera2',
            'gstreamer',
            'v4l2',
        ):
            raise ValueError(
                'Invalid camera_backend'
            )

        if (
            self.c['camera_unique_id']
            and self.c['camera_backend'] not in (
                'auto',
                'picamera2',
            )
        ):
            raise ValueError(
                'camera_unique_id requires picamera2 or auto'
            )

        if self.c['picamera2_format'] not in (
            'RGB888',
            'BGR888',
        ):
            raise ValueError(
                'picamera2_format must be RGB888 or BGR888'
            )

        if (
            self.c['fourcc']
            and len(self.c['fourcc']) != 4
        ):
            raise ValueError(
                'fourcc must contain exactly 4 characters'
            )

        w = self.c['calibration_width']
        h = self.c['calibration_height']

        if (w == 0) != (h == 0):
            raise ValueError(
                'Set both calibration_width '
                'and calibration_height'
            )

        if w and h and not (
            0 <= self.c['intrinsics.cx'] < w
            and 0 <= self.c['intrinsics.cy'] < h
        ):
            raise ValueError(
                'Principal point is outside calibration image'
            )

    # ======================================================
    # 카메라 연결
    # ======================================================
    def _open_source(self):
        self._next_open = (
            time.monotonic()
            + self.c['reopen_interval_sec']
        )

        backend = self.c['camera_backend']

        if backend == 'auto':
            choices = ['picamera2']

            if (
                self.c['gstreamer_pipeline']
                and not self.c['camera_unique_id']
            ):
                choices.append('gstreamer')
        else:
            choices = [backend]

        # auto에서도 CSI 실패 시 임의의 USB 카메라를 열지 않는다.
        # USB 카메라는 camera_backend=v4l2로 명시한다.
        for name in choices:
            source = CameraSource(
                name,
                self.c,
                self.get_logger(),
            )

            if not source.open():
                continue

            try:
                frame = bgr_frame(source.read())

            except Exception as exc:
                source.close()

                self.get_logger().warning(
                    f'[Camera] First frame failed: {exc!r}'
                )

                continue

            self._source = source

            self.get_logger().info(
                f'[Camera] opened: {name}'
            )

            return frame

        raise RuntimeError(
            'No configured camera backend delivered a frame'
        )

    # ======================================================
    # 영상 수신 및 발행
    # ======================================================
    def _on_capture(self):
        try:
            if not self.c['use_camera']:
                frame = self._mock

            elif self._source is None:
                if time.monotonic() < self._next_open:
                    return

                frame = self._open_source()

            else:
                frame = self._source.read()

            frame = bgr_frame(frame)

            # 호스트에서 프레임을 받은 직후의 ROS 시각.
            stamp = self.get_clock().now().to_msg()

            msg = self._bridge.cv2_to_imgmsg(
                frame,
                encoding='bgr8',
            )

            msg.header.stamp = stamp
            msg.header.frame_id = self.c['frame_id']

            info = self._camera_info(frame, stamp)

            self._info_pub.publish(info)
            self._image_pub.publish(msg)

            self._published += 1
            self._last_published = time.monotonic()
            self._consecutive_failures = 0

        except Exception as exc:
            self._capture_failures += 1
            self._consecutive_failures += 1

            self.get_logger().warning(
                f'[Camera] capture/publish failed: {exc!r}',
                throttle_duration_sec=5.0,
            )

            if (
                self._consecutive_failures
                >= self.c['reopen_after_failures']
            ):
                if self._source is not None:
                    self._source.close()
                    self._source = None

                self._next_open = (
                    time.monotonic()
                    + self.c['reopen_interval_sec']
                )

                self._consecutive_failures = 0

    # ======================================================
    # CameraInfo 생성
    # ======================================================
    def _camera_info(self, frame, stamp):
        c = self.c
        h, w = frame.shape[:2]

        if (
            (w, h) != (
                c['image_width'],
                c['image_height'],
            )
            and self._warned_size != (w, h)
        ):
            self._warned_size = (w, h)

            self.get_logger().warning(
                f'[Camera] Actual frame size: {w}x{h}'
            )

        reason = 'calibrated'

        if not c['use_camera']:
            reason = 'mock_image'

        elif (
            c['intrinsics.fx'] <= 0
            or c['intrinsics.fy'] <= 0
        ):
            reason = 'intrinsics_not_set'

        elif (
            c['calibration_width'] == 0
            or c['calibration_height'] == 0
        ):
            reason = 'calibration_size_not_set'

        elif (w, h) != (
            c['calibration_width'],
            c['calibration_height'],
        ):
            reason = 'calibration_size_mismatch'

        elif self._source is None:
            reason = 'camera_not_open'

        elif (
            c['camera_backend'] == 'auto'
            and self._source.backend != 'picamera2'
        ):
            reason = 'fallback_backend_uncalibrated'

        elif (
            self._source.backend == 'picamera2'
            and c['picamera2_autofocus']
        ):
            # 이번 구현에서는 초점이 계속 움직이는 동안
            # 고정 보정값을 거리·자세 추정에 적용하지 않는다.
            reason = 'continuous_autofocus'

        # 프레임마다 새로운 메시지를 만든다.
        info = CameraInfo()

        info.header.stamp = stamp
        info.header.frame_id = c['frame_id']

        info.width = w
        info.height = h
        info.distortion_model = 'plumb_bob'

        info.d = [0.0] * 5
        info.k = [0.0] * 9
        info.r = [0.0] * 9
        info.p = [0.0] * 12

        if reason == 'calibrated':
            fx = c['intrinsics.fx']
            fy = c['intrinsics.fy']
            cx = c['intrinsics.cx']
            cy = c['intrinsics.cy']

            info.d = [
                c[f'distortion.{key}']
                for key in (
                    'k1',
                    'k2',
                    'p1',
                    'p2',
                    'k3',
                )
            ]

            info.k = [
                fx, 0.0, cx,
                0.0, fy, cy,
                0.0, 0.0, 1.0,
            ]

            info.r = [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
            ]

            info.p = [
                fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0,
            ]

        if reason != self._info_reason:
            self._info_reason = reason

            self.get_logger().info(
                f'[Camera] Calibration state: {reason}'
            )

        return info

    # ======================================================
    # 상태 로그
    # ======================================================
    def _on_heartbeat(self):
        now = time.monotonic()

        fps = (
            self._published - self._heartbeat_count
        ) / max(
            now - self._heartbeat_time,
            1e-9,
        )

        self._heartbeat_count = self._published
        self._heartbeat_time = now

        if self._last_published is None:
            age = 'never'
        else:
            age = f'{now - self._last_published:.2f}s'

        backend = (
            self._source.backend
            if self._source
            else 'none'
        )

        self.get_logger().info(
            f'[Camera] alive '
            f'real={self.c["use_camera"]} '
            f'backend={backend} '
            f'fps={fps:.1f} '
            f'failures={self._capture_failures} '
            f'last_frame_age={age} '
            f'calibration={self._info_reason}'
        )

    # ======================================================
    # 종료
    # ======================================================
    def destroy_node(self):
        self._capture_timer.cancel()
        self._heartbeat_timer.cancel()

        if self._source is not None:
            self._source.close()
            self._source = None

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = CameraNode()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()