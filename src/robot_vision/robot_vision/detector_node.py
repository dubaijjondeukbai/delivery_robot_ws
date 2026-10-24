"""YOLO 객체 탐지 / 사람 Pose: 최신 영상만 처리하고 지연된 결과는 폐기한다."""

import copy
import json
import math
import threading
import time
import uuid
from dataclasses import dataclass
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from . import common


@dataclass
class Detection:
    cx: float
    cy: float
    width: float
    height: float
    class_id: int
    class_name: str
    score: float
    track_id: str = ''
    keypoints: Optional[List[List[float]]] = None


class UltralyticsBackend:
    """Only the inference callback may access this object."""

    def __init__(self, node):
        self.node = node
        self.model = None

    def _options(self):
        n = self.node
        return dict(device=n._device, imgsz=n._imgsz,
                    conf=n._conf, iou=n._iou, max_det=n._max_det,
                    classes=n._classes, verbose=False)

    def load(self):
        from ultralytics import YOLO

        if self.node._cpu_threads > 0:
            import torch
            torch.set_num_threads(self.node._cpu_threads)

        model = YOLO(self.node._model_path)
        expected = 'pose' if self.node._pose_mode else 'detect'

        if model.task != expected:
            raise ValueError(
                f'Model task={model.task!r}; expected {expected!r}'
            )

        if self.node._warmup:
            # predict, not track: dummy images must not create tracks.
            size = self.node._imgsz
            model.predict(
                np.zeros((size, size, 3), np.uint8),
                **self._options()
            )

        self.model = model

    def reset_tracker(self):
        predictor = getattr(self.model, 'predictor', None)

        for tracker in getattr(predictor, 'trackers', ()):
            reset = getattr(tracker, 'reset', None)

            if not callable(reset):
                raise RuntimeError(
                    'Installed tracker does not support reset()'
                )

            reset()

    def infer(self, frame):
        results = self.model.track(
            source=frame,
            persist=True,
            tracker='bytetrack.yaml',
            **self._options()
        )

        if not results:
            raise RuntimeError('YOLO returned no Results object')

        result = results[0]
        boxes = result.boxes

        if boxes is None:
            raise RuntimeError('YOLO result has no boxes field')

        if len(boxes) == 0:
            return []

        xywh = boxes.xywh.cpu().numpy()
        scores = boxes.conf.cpu().numpy()
        classes = boxes.cls.cpu().numpy()
        ids = None if boxes.id is None else boxes.id.cpu().numpy()

        points = None

        if self.node._pose_mode:
            keypoints = getattr(result, 'keypoints', None)

            if keypoints is None:
                raise RuntimeError('Pose result has no keypoints')

            points = keypoints.data.cpu().numpy()

            if (
                points.ndim != 3
                or points.shape[0] != len(boxes)
                or points.shape[2] != 3
            ):
                raise ValueError(
                    'Expected pose keypoints with shape N x K x 3'
                )

        detections = []

        for i, box in enumerate(xywh):
            values = [*box, scores[i], classes[i]]

            if not all(math.isfinite(float(v)) for v in values):
                raise ValueError('Non-finite detection')

            cx, cy, width, height = map(float, box)
            score, cls = float(scores[i]), float(classes[i])

            if (
                width <= 0
                or height <= 0
                or not 0 <= score <= 1
                or cls < 0
                or not cls.is_integer()
            ):
                raise ValueError(
                    'Invalid detection box, confidence or class'
                )

            track_id = ''

            if ids is not None:
                identity = float(ids[i])

                if (
                    not math.isfinite(identity)
                    or identity < 0
                    or not identity.is_integer()
                ):
                    raise ValueError('Invalid track ID')

                track_id = str(int(identity))

            keypoints = None

            if points is not None:
                if not np.isfinite(points[i]).all():
                    raise ValueError('Non-finite pose keypoints')

                if np.any(
                    (points[i, :, 2] < 0)
                    | (points[i, :, 2] > 1)
                ):
                    raise ValueError('Invalid keypoint confidence')

                keypoints = points[i].astype(float).tolist()

            class_id = int(cls)
            names = result.names
            class_name = str(names[class_id])

            detections.append(
                Detection(
                    cx,
                    cy,
                    width,
                    height,
                    class_id,
                    class_name,
                    score,
                    track_id,
                    keypoints,
                )
            )

        return detections


def fill_detection2d(det, header, use_class_names):
    msg = Detection2D()
    msg.header = header

    center = msg.bbox.center

    if hasattr(center, 'position'):
        center.position.x, center.position.y = det.cx, det.cy
    else:
        center.x, center.y = det.cx, det.cy

    msg.bbox.size_x, msg.bbox.size_y = det.width, det.height

    hypothesis = ObjectHypothesisWithPose()
    class_id = det.class_name if use_class_names else str(det.class_id)

    if hasattr(hypothesis, 'hypothesis'):
        hypothesis.hypothesis.class_id = class_id
        hypothesis.hypothesis.score = det.score
    else:
        hypothesis.id, hypothesis.score = det.class_id, det.score

    msg.results.append(hypothesis)

    if hasattr(msg, 'id'):
        msg.id = det.track_id

    return msg


class DetectorNode(Node):
    def __init__(self, node_name='detector_node', pose_mode=False):
        super().__init__(node_name)

        self._pose_mode = pose_mode

        defaults = dict(
            enabled=True,
            model_path='yolo11n-pose.pt' if pose_mode else 'yolo11n.pt',
            device='cpu',
            imgsz=416,
            confidence_threshold=0.40,
            iou_threshold=0.50,
            max_detections=20,
            classes=[0] if pose_mode else [-1],
            publish_annotated_image=False,
            heartbeat_period_sec=5.0,
            inference_rate_hz=5.0,
            cpu_threads=0,
            warmup=True,
            use_class_names=False,
            image_topic=common.TOPIC_IMAGE_RAW,
            detections_topic=(
                '/vision/pose_boxes'
                if pose_mode
                else common.TOPIC_DETECTIONS
            ),
            poses_topic='/vision/poses',
            annotated_image_topic=(
                '/vision/pose_debug_image'
                if pose_mode
                else common.TOPIC_DEBUG_IMAGE
            ),
            image_timeout_sec=1.0,
            max_image_age_sec=1.0,
            future_tolerance_sec=0.1,
            tracker_reset_gap_sec=1.0,
            model_retry_sec=5.0,
        )

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        p = lambda name: self.get_parameter(name).value

        self._enabled = bool(p('enabled'))
        self._model_path = str(p('model_path')).strip()
        self._device = str(p('device')).strip()

        if not self._model_path or not self._device:
            raise ValueError('model_path and device must not be empty')

        self._imgsz = self._integer('imgsz', p('imgsz'), 32)
        self._imgsz = int(math.ceil(self._imgsz / 32)) * 32

        self._max_det = self._integer(
            'max_detections', p('max_detections'), 1
        )
        self._cpu_threads = self._integer(
            'cpu_threads', p('cpu_threads'), 0
        )

        self._conf = self._number(
            'confidence_threshold', p('confidence_threshold')
        )
        self._iou = self._number(
            'iou_threshold', p('iou_threshold')
        )

        if self._conf > 1 or self._iou > 1:
            raise ValueError(
                'confidence_threshold and iou_threshold must be <= 1'
            )

        classes = list(p('classes'))

        if any(
            not isinstance(c, int)
            or isinstance(c, bool)
            or c < -1
            for c in classes
        ):
            raise ValueError(
                'classes must contain integer class IDs or -1'
            )

        self._classes = (
            None
            if not classes or -1 in classes
            else sorted(set(classes))
        )

        if pose_mode:
            self._classes = [0]

        self._warmup = bool(p('warmup'))
        self._use_class_names = bool(p('use_class_names'))

        self._max_age = self._number(
            'max_image_age_sec', p('max_image_age_sec')
        )
        self._image_timeout = self._number(
            'image_timeout_sec', p('image_timeout_sec')
        )
        self._future_tolerance = self._number(
            'future_tolerance_sec',
            p('future_tolerance_sec'),
            True,
        )
        self._reset_gap = self._number(
            'tracker_reset_gap_sec', p('tracker_reset_gap_sec')
        )
        self._retry_sec = self._number(
            'model_retry_sec', p('model_retry_sec')
        )

        heartbeat = self._number(
            'heartbeat_period_sec', p('heartbeat_period_sec')
        )
        rate = self._number(
            'inference_rate_hz', p('inference_rate_hz')
        )

        for name in (
            'image_topic',
            'detections_topic',
            'poses_topic',
            'annotated_image_topic',
        ):
            if not str(p(name)).strip():
                raise ValueError(f'{name} must not be empty')

        self._bridge = CvBridge()
        self._lock = threading.Lock()

        self._slot = None
        self._generation = 0
        self._tracking_generation = None
        self._track_epoch = 0
        self._last_nonempty_at = None
        self._run_id = uuid.uuid4().hex[:12]

        self._last_ros_ns = None
        self._source_key = None
        self._source_stamp = None
        self._received_at = None

        self._model_loaded = False
        self._next_load = 0.0
        self._last_result_at = None
        self._last_inference_ms = 0.0

        self._received = 0
        self._published = 0
        self._dropped = 0
        self._errors = 0

        self._io_group = MutuallyExclusiveCallbackGroup()
        self._inference_group = MutuallyExclusiveCallbackGroup()
        self._steady_clock = Clock(
            clock_type=ClockType.STEADY_TIME
        )

        reliable = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._detections_pub = self.create_publisher(
            Detection2DArray,
            p('detections_topic'),
            reliable,
        )

        self._pose_pub = (
            self.create_publisher(
                String, p('poses_topic'), reliable
            )
            if pose_mode
            else None
        )

        self._annotated_pub = (
            self.create_publisher(
                Image, p('annotated_image_topic'), sensor
            )
            if p('publish_annotated_image')
            else None
        )

        self._heartbeat_timer = self.create_timer(
            heartbeat,
            self._on_heartbeat,
            callback_group=self._io_group,
            clock=self._steady_clock,
        )

        self._backend = UltralyticsBackend(self)

        if not self._enabled:
            self.get_logger().warning(
                '[Vision] disabled; no images or model are processed'
            )
            return

        self._image_sub = self.create_subscription(
            Image,
            p('image_topic'),
            self._on_image,
            sensor,
            callback_group=self._io_group,
        )

        self._inference_timer = self.create_timer(
            1.0 / rate,
            self._on_inference_timer,
            callback_group=self._inference_group,
            clock=self._steady_clock,
        )

        self.get_logger().info(
            f'[Vision] ready: model={self._model_path}, '
            f'pose={pose_mode}'
        )

    @staticmethod
    def _number(name, value, allow_zero=False):
        number = float(value)

        if (
            not math.isfinite(number)
            or number < 0
            or (not allow_zero and number == 0)
        ):
            raise ValueError(f'Invalid {name}: {value!r}')

        return number

    @staticmethod
    def _integer(name, value, minimum):
        if (
            isinstance(value, bool)
            or int(value) != value
            or value < minimum
        ):
            raise ValueError(f'Invalid {name}: {value!r}')

        return int(value)

    @staticmethod
    def _stamp_ns(header):
        stamp = header.stamp

        if (
            stamp.sec < 0
            or not 0 <= stamp.nanosec < 1_000_000_000
        ):
            return 0

        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def _fresh(self, header, received):
        stamp = self._stamp_ns(header)
        now = self.get_clock().now().nanoseconds
        age = (now - stamp) * 1e-9

        return (
            stamp > 0
            and now > 0
            and -self._future_tolerance <= age <= self._max_age
            and 0 <= time.monotonic() - received <= self._image_timeout
        )

    def _invalidate(self):
        with self._lock:
            self._slot = None
            self._generation += 1
            self._dropped += 1

    def _on_image(self, msg):
        received = time.monotonic()
        now = self.get_clock().now().nanoseconds

        with self._lock:
            if (
                self._last_ros_ns is not None
                and now < self._last_ros_ns
            ):
                self._slot = None
                self._source_stamp = None
                self._generation += 1

            self._last_ros_ns = now

        if (
            not msg.header.frame_id.strip()
            or not self._fresh(msg.header, received)
        ):
            self._invalidate()
            return

        try:
            frame = common.imgmsg_to_bgr(self._bridge, msg)

            if (
                not isinstance(frame, np.ndarray)
                or frame.dtype != np.uint8
                or frame.ndim != 3
                or frame.shape[2] != 3
                or min(frame.shape[:2]) <= 0
            ):
                raise ValueError(
                    'Expected nonempty uint8 BGR image'
                )

            frame = frame.copy()

        except Exception as exc:
            self._invalidate()

            self.get_logger().error(
                f'[Vision] invalid image: {exc!r}',
                throttle_duration_sec=5.0,
            )
            return

        stamp = self._stamp_ns(msg.header)
        key = (msg.header.frame_id, frame.shape[:2])

        with self._lock:
            changed = key != self._source_key

            if (
                not changed
                and self._source_stamp is not None
                and stamp <= self._source_stamp
            ):
                self._dropped += 1
                return

            gap = (
                self._received_at is not None
                and received - self._received_at > self._reset_gap
            )

            stamp_gap = (
                self._source_stamp is not None
                and (stamp - self._source_stamp) * 1e-9
                > self._reset_gap
            )

            if changed or gap or stamp_gap:
                self._generation += 1

            if self._slot is not None:
                self._dropped += 1

            self._source_key = key
            self._source_stamp = stamp
            self._received_at = received
            self._received += 1

            self._slot = (
                frame,
                copy.deepcopy(msg.header),
                received,
                self._generation,
            )

    def _on_inference_timer(self):
        if not self._model_loaded:
            if time.monotonic() < self._next_load:
                return

            try:
                self._backend = UltralyticsBackend(self)
                self._backend.load()
                self._model_loaded = True
                self._tracking_generation = None

                self.get_logger().info(
                    '[Vision] model loaded'
                )

            except Exception as exc:
                self._next_load = (
                    time.monotonic() + self._retry_sec
                )

                self.get_logger().error(
                    '[Vision] model load failed; '
                    f'retry later: {exc!r}'
                )
                return

        with self._lock:
            slot, self._slot = self._slot, None

        if slot is None:
            return

        frame, header, received, generation = slot

        if not self._fresh(header, received):
            self._tracking_generation = None
            self._dropped += 1
            return

        started = time.monotonic()

        try:
            track_expired = (
                self._last_nonempty_at is not None
                and started - self._last_nonempty_at
                > self._reset_gap
            )

            if (
                generation != self._tracking_generation
                or track_expired
            ):
                self._backend.reset_tracker()
                self._tracking_generation = generation
                self._track_epoch += 1
                self._last_nonempty_at = None

            detections = self._backend.infer(frame)

            if detections:
                self._last_nonempty_at = time.monotonic()

            self._last_inference_ms = (
                time.monotonic() - started
            ) * 1000

            for det in detections:
                if det.track_id:
                    det.track_id = (
                        f'{self._run_id}:'
                        f'{self._track_epoch}:'
                        f'{det.track_id}'
                    )

            array = Detection2DArray()
            array.header = header
            array.detections = [
                fill_detection2d(
                    d, header, self._use_class_names
                )
                for d in detections
            ]

            pose_msg = None

            if self._pose_pub is not None:
                people = [
                    dict(
                        pose_track_id=d.track_id,
                        score=d.score,
                        bbox=[
                            d.cx - d.width / 2,
                            d.cy - d.height / 2,
                            d.cx + d.width / 2,
                            d.cy + d.height / 2,
                        ],
                        keypoints=d.keypoints,
                    )
                    for d in detections
                    if d.class_id == 0
                    and d.keypoints is not None
                ]

                pose_msg = String()
                pose_msg.data = json.dumps(
                    dict(
                        schema=1,
                        stamp=(
                            header.stamp.sec
                            + header.stamp.nanosec * 1e-9
                        ),
                        frame_id=header.frame_id,
                        width=frame.shape[1],
                        height=frame.shape[0],
                        people=people,
                    ),
                    allow_nan=False,
                )

            # Check again after inference.
            # Reject in-flight results from an old stream.
            with self._lock:
                if (
                    generation != self._generation
                    or not self._fresh(header, received)
                ):
                    self._tracking_generation = None
                    self._dropped += 1
                    return

                self._detections_pub.publish(array)

                if pose_msg is not None:
                    self._pose_pub.publish(pose_msg)

                self._last_result_at = time.monotonic()
                self._published += 1

        except Exception as exc:
            self._errors += 1
            self._model_loaded = False
            self._tracking_generation = None
            self._next_load = (
                time.monotonic() + self._retry_sec
            )

            self.get_logger().error(
                '[Vision] inference/output failed; '
                f'model will reload: {exc!r}',
                throttle_duration_sec=5.0,
            )
            return

        if self._annotated_pub is not None:
            self._publish_annotated(
                frame,
                detections,
                header,
                received,
                generation,
            )

    def _publish_annotated(
        self,
        frame,
        detections,
        header,
        received,
        generation,
    ):
        try:
            output = frame.copy()
            height, width = output.shape[:2]

            for d in detections:
                x1 = int(
                    np.clip(
                        d.cx - d.width / 2, 0, width - 1
                    )
                )
                y1 = int(
                    np.clip(
                        d.cy - d.height / 2, 0, height - 1
                    )
                )
                x2 = int(
                    np.clip(
                        d.cx + d.width / 2, 0, width - 1
                    )
                )
                y2 = int(
                    np.clip(
                        d.cy + d.height / 2, 0, height - 1
                    )
                )

                cv2.rectangle(
                    output,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2,
                )

                cv2.putText(
                    output,
                    f'{d.class_name} {d.score:.2f}',
                    (x1, max(12, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

            msg = self._bridge.cv2_to_imgmsg(
                output, encoding='bgr8'
            )
            msg.header = header

            with self._lock:
                if (
                    generation == self._generation
                    and self._fresh(header, received)
                ):
                    self._annotated_pub.publish(msg)

        except Exception as exc:
            self.get_logger().warning(
                f'[Vision] debug image failed: {exc!r}',
                throttle_duration_sec=5.0,
            )

    def _on_heartbeat(self):
        now = time.monotonic()

        with self._lock:
            image_age = (
                None
                if self._received_at is None
                else round(now - self._received_at, 2)
            )

        result_age = (
            None
            if self._last_result_at is None
            else round(now - self._last_result_at, 2)
        )

        self.get_logger().info(
            f'[Vision] enabled={self._enabled} '
            f'model_loaded={self._model_loaded} '
            f'pose={self._pose_mode} '
            f'received={self._received} '
            f'published={self._published} '
            f'dropped={self._dropped} '
            f'errors={self._errors} '
            f'image_age_s={image_age} '
            f'result_age_s={result_age} '
            f'inference_ms={self._last_inference_ms:.0f}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = executor = None

    try:
        node = DetectorNode()

        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if executor is not None:
            executor.shutdown()

        if node is not None:
            node.destroy_node()

        rclpy.try_shutdown()


if __name__ == '__main__':
    main()