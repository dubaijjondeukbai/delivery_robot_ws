#!/usr/bin/env python3
"""
[Vision] detector_node

Role
----
YOLO object detection (Ultralytics) on camera images, published as vision_msgs.

    /camera/image_raw (sensor_msgs/Image)
            |
            v
     detector_node  ---> /vision/detections (Detection2DArray)
                    ---> /vision/debug_image (Image, optional)

[추가] 같은 DetectorNode를 Pose 모드로도 사용한다.
     pose_node      ---> /vision/pose_boxes (Detection2DArray)
                    ---> /vision/poses (String 안의 JSON)
                    ---> /vision/pose_debug_image (Image, optional)

Status
------
IMPLEMENTED. Default model yolo11n.pt, device cpu, imgsz 416 (Raspberry Pi 5 budget).
enabled: false keeps the node running (heartbeat only) without loading a model,
e.g. while torch/ultralytics are not installed on the target yet.
[추가] Pose 모드의 기본 모델은 yolo11n-pose.pt다.

Backlog policy (Raspberry Pi 5)
-------------------------------
The image callback only converts and stores the *latest* frame. A separate
inference timer (inference_rate_hz) takes that frame and runs YOLO. The
subscription and the inference timer live in different mutually-exclusive
callback groups on a 2-thread executor, so frames keep arriving (and superseding
each other) while an inference is running. Frames that are overwritten before
being processed are counted as dropped_frames. A camera at 15 fps with YOLO at
5 fps therefore never builds a queue.

Model interface
---------------
UltralyticsBackend is the only place that touches the ultralytics API. Point
model_path at an exported model (e.g. yolo11n_ncnn_model) for NCNN, or implement
another DetectionBackend subclass for a hardware accelerator; the ROS side does
not change.

Safety / architecture
---------------------
This node publishes perception results only. It never publishes /cmd_vel, STOP
signals or anything that reaches the motor controllers. "person detected" is
information for robot_safety / task_manager, never a command.

Tracking / future
-----------------
[변경] ByteTrack을 사용하며 Detection2D.id에 추적 ID를 저장한다.
[추가] Pose 모델이면 관절 정보도 발행한다. 행동 판정과 정지는 여기서 하지 않는다.
LiDAR-vision fusion is handled separately. No depth camera is planned
(the Raspberry Pi Camera Module 3 Wide is monocular).
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from robot_vision import common


# ==================================================
# TOPIC NAMES
# Defaults live in robot_vision/common.py (single source for every vision node)
# and can be overridden in vision.yaml.
# [추가] 새 Pose 토픽 기본값도 이곳에서 관리하고 파라미터로 변경할 수 있다.
# ==================================================
DEFAULT_IMAGE_TOPIC = common.TOPIC_IMAGE_RAW
DEFAULT_DETECTIONS_TOPIC = common.TOPIC_DETECTIONS
DEFAULT_DEBUG_IMAGE_TOPIC = common.TOPIC_DEBUG_IMAGE

DEFAULT_POSE_BOXES_TOPIC = '/vision/pose_boxes'
DEFAULT_POSES_TOPIC = '/vision/poses'
DEFAULT_POSE_DEBUG_IMAGE_TOPIC = '/vision/pose_debug_image'

# Default COCO classes of interest for a delivery robot
# (person, bicycle, car, motorcycle, bus, truck).
# [변경] 기존 참고 목록은 유지한다.
# 실제 기본 설정은 일반 탐지 [-1], Pose [0]이다.
DEFAULT_CLASSES = [0, 1, 2, 3, 5, 7]

# classes: [-1] means "do not filter by class"
ALL_CLASSES_SENTINEL = -1


# ==================================================
# Backend interface
# ==================================================
@dataclass
class Detection:
    """One detected object in image pixel coordinates (bbox centre + size)."""

    cx: float
    cy: float
    width: float
    height: float
    class_id: int
    class_name: str
    score: float

    # [추가] 같은 객체를 프레임 사이에서 구분하는 추적 ID
    # 아직 ID가 없으면 빈 문자열
    track_id: str = ''

    # [추가] 사람 Pose의 관절 정보
    # 일반 탐지 결과에는 관절이 없으므로 None
    keypoints: Optional[List[List[float]]] = None


class DetectionBackend:
    """Minimal interface every detector backend must implement."""

    name = 'abstract'

    def load(self) -> None:
        raise NotImplementedError

    def infer(self, frame_bgr: np.ndarray) -> List[Detection]:
        raise NotImplementedError


# Detection에 Pose 정보를 추가한다.
# 이 클래스는 관절을 추출하며 행동은 판단하지 않는다.
class UltralyticsBackend(DetectionBackend):
    """Ultralytics YOLO: PyTorch .pt or exported formats."""

    name = 'ultralytics'

    def __init__(
        self,
        model_path: str,
        device: str,
        imgsz: int,
        conf: float,
        iou: float,
        max_det: int,
        classes: Optional[Sequence[int]],
        cpu_threads: int,
        expected_task: str = 'detect',
    ) -> None:
        self._model_path = model_path
        self._device = device
        self._imgsz = imgsz
        self._conf = conf
        self._iou = iou
        self._max_det = max_det
        self._classes = list(classes) if classes else None
        self._cpu_threads = cpu_threads

        # [추가] 일반 탐지는 detect, 사람 Pose는 pose
        self._expected_task = expected_task

        self._model = None
        self._names = {}

    def load(self) -> None:
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                'The ultralytics package is not installed for this Python interpreter. '
                'Install it with: python3 -m pip install ultralytics'
            ) from exc

        if self._cpu_threads > 0:
            try:
                import torch

                torch.set_num_threads(self._cpu_threads)

            except ImportError:
                pass  # exported backends may not need torch at runtime

        # Note: a bare file name such as "yolo11n.pt" is downloaded by ultralytics
        # into the current working directory on first use;
        # prefer an absolute model_path on the robot.
        self._model = YOLO(self._model_path)

        # [추가] YAML이 잘못된 모델을 지정했는지 검사
        # 예: Pose 모드인데 일반 탐지 모델을 지정한 경우
        actual_task = getattr(self._model, 'task', None)

        if actual_task != self._expected_task:
            raise RuntimeError(
                f'Expected a {self._expected_task!r} model, '
                f'got {actual_task!r}: {self._model_path}'
            )

        names = getattr(self._model, 'names', None)
        self._names = dict(names) if isinstance(names, dict) else {}

    def infer(self, frame_bgr: np.ndarray) -> List[Detection]:

        # [변경] 객체 탐지와 ByteTrack 추적을 함께 실행
        results = self._model.track(
            source=frame_bgr,
            persist=True,
            tracker='bytetrack.yaml',
            imgsz=self._imgsz,
            conf=self._conf,
            iou=self._iou,
            max_det=self._max_det,
            classes=self._classes,
            device=self._device,
            verbose=False,
        )

        if not results:
            return []

        result = results[0]
        boxes = result.boxes

        if boxes is None or len(boxes) == 0:
            return []

        # xywh = bbox centre x, centre y, width, height
        # in original image pixels.
        xywh = boxes.xywh.cpu().numpy()
        scores = boxes.conf.cpu().numpy()
        class_ids = boxes.cls.cpu().numpy().astype(int)

        names = (
            result.names
            if isinstance(getattr(result, 'names', None), dict)
            else self._names
        )

        # [추가] 추적 ID 추출
        # 아직 ID가 부여되지 않았다면 빈 문자열 사용
        if boxes.id is not None:
            track_ids = [
                str(int(value))
                for value in boxes.id.cpu().numpy()
            ]
        else:
            track_ids = [''] * len(boxes)

        # [추가] Pose 모델이면 관절 정보를 가져온다.
        # 일반 객체 탐지 모델에는 관절 정보가 없으므로 None
        pose_result = getattr(result, 'keypoints', None)

        if pose_result is not None:
            keypoints = pose_result.data.cpu().numpy()
        else:
            keypoints = None

        detections: List[Detection] = []

        for index, ((cx, cy, w, h), score, class_id) in enumerate(
            zip(xywh, scores, class_ids)
        ):
            detections.append(
                Detection(
                    cx=float(cx),
                    cy=float(cy),
                    width=float(w),
                    height=float(h),
                    class_id=int(class_id),
                    class_name=str(
                        names.get(int(class_id), int(class_id))
                    ),
                    score=float(score),

                    # [추가] dataclass에 실제 결과 저장
                    track_id=track_ids[index],
                    keypoints=(
                        keypoints[index].tolist()
                        if keypoints is not None
                        else None
                    ),
                )
            )

        return detections


# ==================================================
# ROS message helpers (tolerant to vision_msgs API versions)
# ==================================================
def fill_detection2d(
    msg: Detection2D,
    det: Detection,
    header: Header,
    use_class_names: bool,
) -> Detection2D:
    """Fill a Detection2D. Handles two BoundingBox2D.center layouts."""

    msg.header = header

    center = msg.bbox.center

    if hasattr(center, 'position'):
        # vision_msgs >= 4.1: vision_msgs/Pose2D {position, theta}
        center.position.x = det.cx
        center.position.y = det.cy
    else:
        # vision_msgs 4.0: geometry_msgs/Pose2D {x, y, theta}
        center.x = det.cx
        center.y = det.cy

    center.theta = 0.0
    msg.bbox.size_x = det.width
    msg.bbox.size_y = det.height

    hypothesis = ObjectHypothesisWithPose()

    class_id = (
        det.class_name
        if use_class_names
        else str(det.class_id)
    )

    if hasattr(hypothesis, 'hypothesis'):
        # vision_msgs >= 4.0
        hypothesis.hypothesis.class_id = class_id
        hypothesis.hypothesis.score = det.score
    else:
        # vision_msgs 3.x (Foxy)
        hypothesis.id = class_id
        hypothesis.score = det.score

    msg.results.append(hypothesis)

    # [변경] ByteTrack 추적 ID를 ROS 메시지에 저장
    # 아직 ID가 없으면 빈 문자열이 들어간다.
    # 데이터 저장 순서:
    # ByteTrack ID → Detection.track_id → ROS 메시지의 id
    msg.id = det.track_id

    return msg


def draw_detections(
    frame_bgr: np.ndarray,
    detections: List[Detection],
) -> np.ndarray:
    """Annotate boxes and labels onto a copy of the frame."""

    out = frame_bgr.copy()

    for det in detections:
        x1 = int(det.cx - det.width / 2.0)
        y1 = int(det.cy - det.height / 2.0)
        x2 = int(det.cx + det.width / 2.0)
        y2 = int(det.cy + det.height / 2.0)

        cv2.rectangle(
            out,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2,
        )

        cv2.putText(
            out,
            f'{det.class_name} {det.score:.2f}',
            (x1, max(y1 - 5, 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

    return out


# ==================================================
# Node
# ==================================================
class DetectorNode(Node):
    """YOLO detector with latest-frame scheduling."""

    def __init__(
        self,
        node_name: str = 'detector_node',
        pose_mode: bool = False,
    ) -> None:
        super().__init__(node_name)

        # 노드 이름과 별개로 Pose 사용 여부 설정
        self._pose_mode = pose_mode

        self._bridge = CvBridge()
        self._backend = None  # type: Optional[DetectionBackend]
        self._model_loaded = False

        # Latest-frame slot shared between the image callback
        # and the inference timer.
        self._frame_lock = threading.Lock()
        self._latest_frame = None  # type: Optional[np.ndarray]
        self._latest_header = None  # type: Optional[Header]

        # Heartbeat statistics.
        self._frames_received = 0
        self._inferences = 0
        self._inferences_at_last_heartbeat = 0
        self._detections_total = 0
        self._dropped_frames = 0
        self._inference_errors = 0
        self._conversion_failures = 0
        self._last_inference_ms = 0.0

        self._declare_parameters()
        self._load_parameters()

        # subscription + heartbeat
        self._io_group = MutuallyExclusiveCallbackGroup()

        # inference timer
        self._inference_group = MutuallyExclusiveCallbackGroup()

        self._detections_pub = self.create_publisher(
            Detection2DArray,
            self._detections_topic,
            10,
        )

        self._annotated_pub = None

        # [추가] 사람 Pose 모드에서만 관절 정보 Publisher 생성
        self._pose_pub = None

        # _detections_pub: 객체 또는 사람의 박스·종류·추적 ID
        # _pose_pub: 사람의 관절 정보 — Pose 모드에서만 사용
        if self._pose_mode:
            self._pose_pub = self.create_publisher(
                String,
                self._poses_topic,
                10,
            )

        if self._publish_annotated_image:
            self._annotated_pub = self.create_publisher(
                Image,
                self._annotated_image_topic,
                qos_profile_sensor_data,
            )

        self._heartbeat_timer = self.create_timer(
            self._heartbeat_period_sec,
            self._on_heartbeat,
            callback_group=self._io_group,
        )

        if not self._enabled:
            self.get_logger().warning(
                '[Vision] node is disabled (enabled: false). '
                'No model is loaded and no detections will be published; '
                'heartbeat only.'
            )
            return

        self._backend = UltralyticsBackend(
            model_path=self._model_path,
            device=self._device,
            imgsz=self._imgsz,
            conf=self._confidence_threshold,
            iou=self._iou_threshold,
            max_det=self._max_detections,
            classes=self._classes,
            cpu_threads=self._cpu_threads,
            expected_task='pose' if self._pose_mode else 'detect',
        )

        self._load_model()

        self._image_sub = self.create_subscription(
            Image,
            self._image_topic,
            self._on_image,
            qos_profile_sensor_data,
            callback_group=self._io_group,
        )

        self._inference_timer = self.create_timer(
            1.0 / self._inference_rate_hz,
            self._on_inference_timer,
            callback_group=self._inference_group,
        )

        self.get_logger().info(
            f'[Vision] {node_name} ready: '
            f'backend={self._backend.name} '
            f'model={self._model_path} '
            f'pose_mode={self._pose_mode} '
            f'device={self._device} '
            f'imgsz={self._imgsz} '
            f'conf={self._confidence_threshold} '
            f'iou={self._iou_threshold} '
            f'classes={self._classes if self._classes else "all"} '
            f'inference_rate_hz={self._inference_rate_hz} '
            f'annotated_image={self._publish_annotated_image}'
        )

    # --------------------------------------------------
    # Parameters
    # --------------------------------------------------
    def _declare_parameters(self) -> None:
        self.declare_parameter('enabled', True)

        # [변경] 사람 Pose / 일반 객체 탐지 모델 선택
        self.declare_parameter(
            'model_path',
            'yolo11n-pose.pt' if self._pose_mode else 'yolo11n.pt',
        )

        self.declare_parameter('device', 'cpu')
        self.declare_parameter('imgsz', 416)
        self.declare_parameter('confidence_threshold', 0.40)
        self.declare_parameter('iou_threshold', 0.50)
        self.declare_parameter('max_detections', 20)

        # [변경] Pose는 사람(0)만,
        # 일반 탐지는 모델이 지원하는 전체 클래스
        self.declare_parameter(
            'classes',
            [0] if self._pose_mode else [-1],
        )

        self.declare_parameter('publish_annotated_image', False)

        # [변경] 일반 탐지와 Pose의 디버그 영상 토픽 구분
        self.declare_parameter(
            'annotated_image_topic',
            DEFAULT_POSE_DEBUG_IMAGE_TOPIC
            if self._pose_mode
            else DEFAULT_DEBUG_IMAGE_TOPIC,
        )

        self.declare_parameter('heartbeat_period_sec', 5.0)
        self.declare_parameter('inference_rate_hz', 5.0)

        # 0 = library default
        self.declare_parameter('cpu_threads', 0)

        self.declare_parameter('warmup', True)

        # class_id "person" instead of "0"
        self.declare_parameter('use_class_names', False)

        # 두 모델이 같은 카메라 영상을 입력으로 사용
        self.declare_parameter(
            'image_topic',
            DEFAULT_IMAGE_TOPIC,
        )

        # [변경] 일반 탐지 결과와 Pose 박스의 출력 토픽 구분
        self.declare_parameter(
            'detections_topic',
            DEFAULT_POSE_BOXES_TOPIC
            if self._pose_mode
            else DEFAULT_DETECTIONS_TOPIC,
        )

        # [추가] 관절 JSON 출력 토픽도 설정으로 변경할 수 있다.
        self.declare_parameter(
            'poses_topic',
            DEFAULT_POSES_TOPIC,
        )

    def _load_parameters(self) -> None:
        log = self.get_logger()
        p = self.get_parameter

        # [변경] 비어 있거나 잘못된 설정의 대체값도
        # 모드에 맞게 선택한다.
        default_model = (
            'yolo11n-pose.pt'
            if self._pose_mode
            else 'yolo11n.pt'
        )

        default_classes = [0] if self._pose_mode else [-1]

        default_detections = (
            DEFAULT_POSE_BOXES_TOPIC
            if self._pose_mode
            else DEFAULT_DETECTIONS_TOPIC
        )

        default_debug = (
            DEFAULT_POSE_DEBUG_IMAGE_TOPIC
            if self._pose_mode
            else DEFAULT_DEBUG_IMAGE_TOPIC
        )

        self._enabled = bool(p('enabled').value)

        self._model_path = str(p('model_path').value).strip()

        if not self._model_path:
            log.error(
                "[Vision] Parameter 'model_path' is empty; "
                f'using {default_model!r}'
            )
            self._model_path = default_model

        self._device = str(p('device').value).strip() or 'cpu'

        self._imgsz = int(
            common.positive_or_default(
                log,
                'imgsz',
                p('imgsz').value,
                416,
            )
        )

        if self._imgsz % 32 != 0:
            adjusted = max(
                32,
                int(round(self._imgsz / 32.0)) * 32,
            )

            log.warning(
                f'[Vision] imgsz={self._imgsz} '
                f'is not a multiple of 32; using {adjusted}'
            )

            self._imgsz = adjusted

        conf = float(p('confidence_threshold').value)

        if not 0.0 < conf <= 1.0:
            log.error(
                "[Vision] Parameter "
                f"'confidence_threshold'={conf} "
                'must be in (0, 1]; using 0.40'
            )
            conf = 0.40

        self._confidence_threshold = conf

        iou = float(p('iou_threshold').value)

        if not 0.0 < iou <= 1.0:
            log.error(
                f"[Vision] Parameter 'iou_threshold'={iou} "
                'must be in (0, 1]; using 0.50'
            )
            iou = 0.50

        self._iou_threshold = iou

        self._max_detections = int(
            common.positive_or_default(
                log,
                'max_detections',
                p('max_detections').value,
                20,
            )
        )

        raw_classes = p('classes').value

        classes = [
            int(c)
            for c in (
                raw_classes
                if raw_classes is not None
                else []
            )
        ]

        if not classes:
            classes = list(default_classes)

        elif any(
            c < 0 and c != ALL_CLASSES_SENTINEL
            for c in classes
        ):
            log.error(
                f'[Vision] Invalid classes={classes}; '
                f'using {default_classes}'
            )
            classes = list(default_classes)

        # [추가] 사람용 사전학습 Pose 모델은
        # person 클래스만 대상으로 한다.
        if self._pose_mode and classes != [0]:
            log.warning(
                '[Vision] Person pose model requires classes=[0]; '
                f'overriding {classes}'
            )
            classes = [0]

        if ALL_CLASSES_SENTINEL in classes:
            self._classes = None  # no class filter
        else:
            self._classes = classes

        self._publish_annotated_image = bool(
            p('publish_annotated_image').value
        )

        self._annotated_image_topic = (
            str(p('annotated_image_topic').value).strip()
            or default_debug
        )

        self._heartbeat_period_sec = float(
            common.positive_or_default(
                log,
                'heartbeat_period_sec',
                p('heartbeat_period_sec').value,
                5.0,
            )
        )

        self._inference_rate_hz = float(
            common.positive_or_default(
                log,
                'inference_rate_hz',
                p('inference_rate_hz').value,
                5.0,
            )
        )

        self._cpu_threads = max(
            0,
            int(p('cpu_threads').value),
        )

        self._warmup = bool(p('warmup').value)
        self._use_class_names = bool(p('use_class_names').value)

        self._image_topic = (
            str(p('image_topic').value).strip()
            or DEFAULT_IMAGE_TOPIC
        )

        self._detections_topic = (
            str(p('detections_topic').value).strip()
            or default_detections
        )

        self._poses_topic = (
            str(p('poses_topic').value).strip()
            or DEFAULT_POSES_TOPIC
        )

    # --------------------------------------------------
    # Model
    # --------------------------------------------------
    def _load_model(self) -> None:
        """Load the model once. On failure the node stays alive."""

        started = time.monotonic()

        try:
            self._backend.load()

        except Exception as exc:  # noqa: BLE001
            self._model_loaded = False

            self.get_logger().error(
                f'[Vision] YOLO model load FAILED ({exc!r}). '
                'Node keeps running but publishes nothing. '
                f'Check model_path={self._model_path!r}, '
                'the ultralytics install and model task.'
            )
            return

        self._model_loaded = True

        self.get_logger().info(
            '[Vision] YOLO model loaded in '
            f'{time.monotonic() - started:.1f} s: '
            f'{self._model_path}'
        )

        if self._warmup:
            # First inference is slow (lazy initialisation);
            # do it now, off the critical path.
            try:
                dummy = np.zeros(
                    (self._imgsz, self._imgsz, 3),
                    dtype=np.uint8,
                )

                t0 = time.monotonic()
                self._backend.infer(dummy)

                self.get_logger().info(
                    '[Vision] Warm-up inference done in '
                    f'{(time.monotonic() - t0) * 1000.0:.0f} ms'
                )

            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(
                    f'[Vision] Warm-up inference failed: {exc!r}'
                )

    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------
    def _on_image(self, msg: Image) -> None:
        """Store the newest frame only. Never blocks on inference."""

        self._frames_received += 1

        try:
            frame = common.imgmsg_to_bgr(
                self._bridge,
                msg,
            )

        except (CvBridgeError, cv2.error) as exc:
            self._conversion_failures += 1

            self.get_logger().error(
                f'[Vision] cv_bridge conversion failed: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        with self._frame_lock:
            if self._latest_frame is not None:
                # previous frame was never inferred
                self._dropped_frames += 1

            self._latest_frame = frame
            self._latest_header = msg.header

    # [변경] _on_image()와 같은 들여쓰기 깊이의
    # 독립된 클래스 메서드다.
    def _on_inference_timer(self) -> None:
        if not self._model_loaded:
            return

        # 기존: 가장 최신 프레임을 가져오고 저장 공간 비우기
        with self._frame_lock:
            frame = self._latest_frame
            header = self._latest_header

            self._latest_frame = None
            self._latest_header = None

        if frame is None or header is None:
            return  # no new frame since the last inference

        t0 = time.monotonic()

        try:
            detections = self._backend.infer(frame)

        except Exception as exc:  # noqa: BLE001
            self._inference_errors += 1

            self.get_logger().error(
                f'[Vision] YOLO inference failed: {exc!r}',
                throttle_duration_sec=5.0,
            )
            return

        # 기존: 추론 시간과 처리 통계 갱신
        self._last_inference_ms = (
            time.monotonic() - t0
        ) * 1000.0

        self._inferences += 1
        self._detections_total += len(detections)

        # Always publish, even when empty:
        # downstream can tell "nothing seen" from "detector dead".
        array = Detection2DArray()
        array.header = header

        for det in detections:
            array.detections.append(
                fill_detection2d(
                    Detection2D(),
                    det,
                    header,
                    self._use_class_names,
                )
            )

        self._detections_pub.publish(array)

        # [추가] Pose 모드에서는 사람의 관절 정보도 발행
        if self._pose_pub is not None:

            # 추론 완료 시각 대신 원본 영상의 촬영 시각을 유지
            payload = {
                'schema': 1,
                'stamp': (
                    header.stamp.sec
                    + header.stamp.nanosec * 1e-9
                ),
                'frame_id': header.frame_id,
                'width': int(frame.shape[1]),
                'height': int(frame.shape[0]),
                'people': [],
            }

            for det in detections:
                # 사람이며 관절 정보가 있는 결과만 저장
                if det.class_id != 0 or det.keypoints is None:
                    continue

                payload['people'].append({
                    'pose_track_id': det.track_id,
                    'score': det.score,

                    # 중심점·크기 → 박스 모서리 좌표
                    'bbox': [
                        det.cx - det.width / 2.0,
                        det.cy - det.height / 2.0,
                        det.cx + det.width / 2.0,
                        det.cy + det.height / 2.0,
                    ],

                    # 각 관절의 [x, y, confidence]
                    'keypoints': det.keypoints,
                })

            try:
                pose_msg = String()
                pose_msg.data = json.dumps(
                    payload,
                    allow_nan=False,
                )

                self._pose_pub.publish(pose_msg)

            except (TypeError, ValueError) as exc:
                self.get_logger().warning(
                    f'[Vision] Pose serialization failed: {exc}',
                    throttle_duration_sec=5.0,
                )

        # 기존: 설정이 켜져 있으면 디버그 영상 발행
        if self._annotated_pub is not None:
            self._publish_annotated(
                frame,
                detections,
                header,
            )

    def _publish_annotated(
        self,
        frame: np.ndarray,
        detections: List[Detection],
        header: Header,
    ) -> None:
        try:
            msg = self._bridge.cv2_to_imgmsg(
                draw_detections(frame, detections),
                encoding='bgr8',
            )

        except (CvBridgeError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f'[Vision] Annotated image conversion failed: {exc}',
                throttle_duration_sec=10.0,
            )
            return

        msg.header = header
        self._annotated_pub.publish(msg)

    def _on_heartbeat(self) -> None:
        inferences_since_last = (
            self._inferences
            - self._inferences_at_last_heartbeat
        )

        self._inferences_at_last_heartbeat = self._inferences

        self.get_logger().info(
            f'[Vision] YOLO alive | '
            f'enabled={self._enabled} | '
            f'model_loaded={self._model_loaded} | '
            f'pose_mode={self._pose_mode} | '
            f'frames_received={self._frames_received} | '
            f'inferences={self._inferences} | '
            f'detections={self._detections_total} | '
            f'dropped_frames={self._dropped_frames} | '
            f'inference_errors={self._inference_errors} | '
            f'conversion_failures={self._conversion_failures} | '
            f'inference_hz='
            f'{inferences_since_last / self._heartbeat_period_sec:.1f} | '
            f'last_inference_ms={self._last_inference_ms:.0f}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None
    executor = None

    try:
        node = DetectorNode()

        # Two threads:
        # one keeps swallowing camera frames while the other runs inference.
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    except Exception as exc:  # noqa: BLE001
        get_logger('detector_node').fatal(
            '[Vision] detector_node failed to start or crashed: '
            f'{exc!r}'
        )
        raise

    finally:
        if executor is not None:
            executor.shutdown()

        if node is not None:
            node.destroy_node()

        rclpy.try_shutdown()


if __name__ == '__main__':
    main()