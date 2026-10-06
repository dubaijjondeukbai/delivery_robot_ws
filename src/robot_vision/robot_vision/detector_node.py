#!/usr/bin/env python3
"""
[Vision] detector_node

Role
----
YOLO object detection (Ultralytics) on camera images, published as vision_msgs.

    /camera/image_raw (sensor_msgs/Image)
            |
            v
     detector_node  --->  /vision/detections  (vision_msgs/Detection2DArray, published for EVERY
                    |                          inference, even when empty)
                    --->  /vision/debug_image  (sensor_msgs/Image, only when publish_annotated_image: true)

Status
------
IMPLEMENTED. Default model yolo11n.pt, device cpu, imgsz 416 (Raspberry Pi 5 budget).
enabled: false keeps the node running (heartbeat only) without loading a model,
e.g. while torch/ultralytics are not installed on the target yet.

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

Future
------
ByteTrack (Detection2D.id is reserved for the track id), LiDAR-vision fusion. No depth camera is planned
(the Raspberry Pi Camera Module 3 Wide is monocular).
"""

from __future__ import annotations

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
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from robot_vision import common

# ==================================================
# TOPIC NAMES
# Defaults live in robot_vision/common.py (single source for every vision node)
# and can be overridden in vision.yaml. Do NOT hard-code topic strings anywhere
# else in this file.
# ==================================================
DEFAULT_IMAGE_TOPIC = common.TOPIC_IMAGE_RAW
DEFAULT_DETECTIONS_TOPIC = common.TOPIC_DETECTIONS
DEFAULT_DEBUG_IMAGE_TOPIC = common.TOPIC_DEBUG_IMAGE

# Default COCO classes of interest for a delivery robot (person, bicycle, car, motorcycle, bus, truck).
DEFAULT_CLASSES = [0, 1, 2, 3, 5, 7]
ALL_CLASSES_SENTINEL = -1  # classes: [-1] in vision.yaml means "do not filter by class"


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


class DetectionBackend:
    """Minimal interface every detector backend must implement."""

    name = 'abstract'

    def load(self) -> None:
        raise NotImplementedError

    def infer(self, frame_bgr: np.ndarray) -> List[Detection]:
        raise NotImplementedError


class UltralyticsBackend(DetectionBackend):
    """Ultralytics YOLO (PyTorch .pt, or exported formats such as *_ncnn_model, .onnx)."""

    name = 'ultralytics'

    def __init__(self, model_path: str, device: str, imgsz: int, conf: float, iou: float,
                 max_det: int, classes: Optional[Sequence[int]], cpu_threads: int) -> None:
        self._model_path = model_path
        self._device = device
        self._imgsz = imgsz
        self._conf = conf
        self._iou = iou
        self._max_det = max_det
        self._classes = list(classes) if classes else None
        self._cpu_threads = cpu_threads
        self._model = None
        self._names = {}

    def load(self) -> None:
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                'The ultralytics package is not installed for this Python interpreter. '
                'Install it with: python3 -m pip install ultralytics (see README).') from exc

        if self._cpu_threads > 0:
            try:
                import torch
                torch.set_num_threads(self._cpu_threads)
            except ImportError:
                pass  # exported backends may not need torch at runtime

        # Note: a bare file name such as "yolo11n.pt" is downloaded by ultralytics into the
        # current working directory on first use; prefer an absolute model_path on the robot.
        self._model = YOLO(self._model_path)
        names = getattr(self._model, 'names', None)
        self._names = dict(names) if isinstance(names, dict) else {}

    def infer(self, frame_bgr: np.ndarray) -> List[Detection]:
        results = self._model.predict(
            source=frame_bgr,
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

        # xywh = bbox centre x, centre y, width, height in original image pixels.
        xywh = boxes.xywh.cpu().numpy()
        scores = boxes.conf.cpu().numpy()
        class_ids = boxes.cls.cpu().numpy().astype(int)
        names = result.names if isinstance(getattr(result, 'names', None), dict) else self._names

        detections: List[Detection] = []
        for (cx, cy, w, h), score, class_id in zip(xywh, scores, class_ids):
            detections.append(Detection(
                cx=float(cx), cy=float(cy), width=float(w), height=float(h),
                class_id=int(class_id), class_name=str(names.get(int(class_id), int(class_id))),
                score=float(score)))
        return detections


# ==================================================
# ROS message helpers (tolerant to vision_msgs API versions)
# ==================================================
def fill_detection2d(msg: Detection2D, det: Detection, header: Header, use_class_names: bool) -> Detection2D:
    """Fill a Detection2D. Handles the two BoundingBox2D.center layouts in vision_msgs 4.x."""
    msg.header = header

    center = msg.bbox.center
    if hasattr(center, 'position'):          # vision_msgs >= 4.1: vision_msgs/Pose2D {position, theta}
        center.position.x = det.cx
        center.position.y = det.cy
    else:                                    # vision_msgs 4.0: geometry_msgs/Pose2D {x, y, theta}
        center.x = det.cx
        center.y = det.cy
    center.theta = 0.0
    msg.bbox.size_x = det.width
    msg.bbox.size_y = det.height

    hypothesis = ObjectHypothesisWithPose()
    class_id = det.class_name if use_class_names else str(det.class_id)
    if hasattr(hypothesis, 'hypothesis'):    # vision_msgs >= 4.0
        hypothesis.hypothesis.class_id = class_id
        hypothesis.hypothesis.score = det.score
    else:                                    # vision_msgs 3.x (Foxy)
        hypothesis.id = class_id
        hypothesis.score = det.score
    msg.results.append(hypothesis)
    # msg.id is intentionally left empty: it is reserved for the tracking id once ByteTrack is added.
    return msg


def draw_detections(frame_bgr: np.ndarray, detections: List[Detection]) -> np.ndarray:
    """Annotate boxes and labels onto a copy of the frame (backend independent)."""
    out = frame_bgr.copy()
    for det in detections:
        x1 = int(det.cx - det.width / 2.0)
        y1 = int(det.cy - det.height / 2.0)
        x2 = int(det.cx + det.width / 2.0)
        y2 = int(det.cy + det.height / 2.0)
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(out, f'{det.class_name} {det.score:.2f}', (x1, max(y1 - 5, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    return out


# ==================================================
# Node
# ==================================================
class DetectorNode(Node):
    """YOLO detector with latest-frame scheduling."""

    def __init__(self) -> None:
        super().__init__('detector_node')

        self._bridge = CvBridge()
        self._backend = None  # type: Optional[DetectionBackend]
        self._model_loaded = False

        # Latest-frame slot shared between the image callback and the inference timer.
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

        self._io_group = MutuallyExclusiveCallbackGroup()         # subscription + heartbeat
        self._inference_group = MutuallyExclusiveCallbackGroup()  # inference timer

        self._detections_pub = self.create_publisher(Detection2DArray, self._detections_topic, 10)
        self._annotated_pub = None
        if self._publish_annotated_image:
            self._annotated_pub = self.create_publisher(Image, self._annotated_image_topic,
                                                        qos_profile_sensor_data)

        self._heartbeat_timer = self.create_timer(self._heartbeat_period_sec, self._on_heartbeat,
                                                  callback_group=self._io_group)

        if not self._enabled:
            self.get_logger().warning('[Vision] detector_node is disabled (enabled: false). No model is loaded and '
                                      'no detections will be published; heartbeat only.')
            return

        self._backend = UltralyticsBackend(
            model_path=self._model_path, device=self._device, imgsz=self._imgsz,
            conf=self._confidence_threshold, iou=self._iou_threshold, max_det=self._max_detections,
            classes=self._classes, cpu_threads=self._cpu_threads)
        self._load_model()

        self._image_sub = self.create_subscription(Image, self._image_topic, self._on_image,
                                                   qos_profile_sensor_data, callback_group=self._io_group)
        self._inference_timer = self.create_timer(1.0 / self._inference_rate_hz, self._on_inference_timer,
                                                  callback_group=self._inference_group)

        self.get_logger().info(
            f'[Vision] detector_node ready: backend={self._backend.name} model={self._model_path} device={self._device} '
            f'imgsz={self._imgsz} conf={self._confidence_threshold} iou={self._iou_threshold} '
            f'classes={self._classes if self._classes else "all"} inference_rate_hz={self._inference_rate_hz} '
            f'annotated_image={self._publish_annotated_image}')

    # --------------------------------------------------
    # Parameters
    # --------------------------------------------------
    def _declare_parameters(self) -> None:
        self.declare_parameter('enabled', True)
        self.declare_parameter('model_path', 'yolo11n.pt')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('imgsz', 416)
        self.declare_parameter('confidence_threshold', 0.40)
        self.declare_parameter('iou_threshold', 0.50)
        self.declare_parameter('max_detections', 20)
        self.declare_parameter('classes', DEFAULT_CLASSES)
        self.declare_parameter('publish_annotated_image', False)
        self.declare_parameter('annotated_image_topic', DEFAULT_DEBUG_IMAGE_TOPIC)
        self.declare_parameter('heartbeat_period_sec', 5.0)
        self.declare_parameter('inference_rate_hz', 5.0)
        self.declare_parameter('cpu_threads', 0)            # 0 = library default
        self.declare_parameter('warmup', True)
        self.declare_parameter('use_class_names', False)    # class_id "person" instead of "0"
        self.declare_parameter('image_topic', DEFAULT_IMAGE_TOPIC)
        self.declare_parameter('detections_topic', DEFAULT_DETECTIONS_TOPIC)

    def _load_parameters(self) -> None:
        log = self.get_logger()
        p = self.get_parameter
        self._enabled = bool(p('enabled').value)
        self._model_path = str(p('model_path').value).strip()
        if not self._model_path:
            log.error("[Vision] Parameter 'model_path' is empty; using 'yolo11n.pt'")
            self._model_path = 'yolo11n.pt'
        self._device = str(p('device').value).strip() or 'cpu'
        self._imgsz = int(common.positive_or_default(log, 'imgsz', p('imgsz').value, 416))
        if self._imgsz % 32 != 0:
            adjusted = max(32, int(round(self._imgsz / 32.0)) * 32)
            log.warning(f'[Vision] imgsz={self._imgsz} is not a multiple of 32; using {adjusted}')
            self._imgsz = adjusted

        conf = float(p('confidence_threshold').value)
        if not 0.0 < conf <= 1.0:
            log.error(f"[Vision] Parameter 'confidence_threshold'={conf} must be in (0, 1]; using 0.40")
            conf = 0.40
        self._confidence_threshold = conf

        iou = float(p('iou_threshold').value)
        if not 0.0 < iou <= 1.0:
            log.error(f"[Vision] Parameter 'iou_threshold'={iou} must be in (0, 1]; using 0.50")
            iou = 0.50
        self._iou_threshold = iou

        self._max_detections = int(common.positive_or_default(log, 'max_detections', p('max_detections').value, 20))

        raw_classes = p('classes').value
        classes = [int(c) for c in (raw_classes if raw_classes is not None else [])]
        if ALL_CLASSES_SENTINEL in classes:
            self._classes = None   # no class filter
        elif any(c < 0 for c in classes):
            log.error(f"[Vision] Parameter 'classes'={classes} contains negative ids; using {DEFAULT_CLASSES}")
            self._classes = list(DEFAULT_CLASSES)
        else:
            self._classes = classes if classes else list(DEFAULT_CLASSES)

        self._publish_annotated_image = bool(p('publish_annotated_image').value)
        self._annotated_image_topic = str(p('annotated_image_topic').value) or DEFAULT_DEBUG_IMAGE_TOPIC
        self._heartbeat_period_sec = float(common.positive_or_default(
            log, 'heartbeat_period_sec', p('heartbeat_period_sec').value, 5.0))
        self._inference_rate_hz = float(common.positive_or_default(
            log, 'inference_rate_hz', p('inference_rate_hz').value, 5.0))
        self._cpu_threads = max(0, int(p('cpu_threads').value))
        self._warmup = bool(p('warmup').value)
        self._use_class_names = bool(p('use_class_names').value)
        self._image_topic = str(p('image_topic').value) or DEFAULT_IMAGE_TOPIC
        self._detections_topic = str(p('detections_topic').value) or DEFAULT_DETECTIONS_TOPIC

    # --------------------------------------------------
    # Model
    # --------------------------------------------------
    def _load_model(self) -> None:
        """Load the model once. On failure the node stays alive and says so loudly."""
        started = time.monotonic()
        try:
            self._backend.load()
        except Exception as exc:  # noqa: BLE001 - ultralytics/torch raise many exception types
            self._model_loaded = False
            self.get_logger().error(
                f'[Vision] YOLO model load FAILED ({exc!r}). detector_node keeps running but publishes nothing. '
                f'Check model_path={self._model_path!r}, the ultralytics install and network access for '
                f'the first-time weight download.')
            return
        self._model_loaded = True
        self.get_logger().info(f'[Vision] YOLO model loaded in {time.monotonic() - started:.1f} s: {self._model_path}')

        if self._warmup:
            # First inference is slow (lazy initialisation); do it now, off the critical path.
            try:
                dummy = np.zeros((self._imgsz, self._imgsz, 3), dtype=np.uint8)
                t0 = time.monotonic()
                self._backend.infer(dummy)
                self.get_logger().info(f'[Vision] Warm-up inference done in {(time.monotonic() - t0) * 1000.0:.0f} ms')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f'[Vision] Warm-up inference failed: {exc!r}')

    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------
    def _on_image(self, msg: Image) -> None:
        """Store the newest frame only. Never blocks on inference."""
        self._frames_received += 1
        try:
            frame = common.imgmsg_to_bgr(self._bridge, msg)
        except (CvBridgeError, cv2.error) as exc:
            self._conversion_failures += 1
            self.get_logger().error(f'[Vision] cv_bridge conversion failed: {exc}', throttle_duration_sec=5.0)
            return
        with self._frame_lock:
            if self._latest_frame is not None:
                self._dropped_frames += 1   # previous frame was never inferred
            self._latest_frame = frame
            self._latest_header = msg.header

    def _on_inference_timer(self) -> None:
        if not self._model_loaded:
            return
        with self._frame_lock:
            frame, header = self._latest_frame, self._latest_header
            self._latest_frame = None
            self._latest_header = None
        if frame is None:
            return  # no new frame since the last inference

        t0 = time.monotonic()
        try:
            detections = self._backend.infer(frame)
        except Exception as exc:  # noqa: BLE001 - keep the node alive on any inference error
            self._inference_errors += 1
            self.get_logger().error(f'[Vision] YOLO inference failed: {exc!r}', throttle_duration_sec=5.0)
            return
        self._last_inference_ms = (time.monotonic() - t0) * 1000.0
        self._inferences += 1
        self._detections_total += len(detections)

        # Always publish, even when empty: downstream can tell "nothing seen" from "detector dead".
        array = Detection2DArray()
        array.header = header
        for det in detections:
            array.detections.append(fill_detection2d(Detection2D(), det, header, self._use_class_names))
        self._detections_pub.publish(array)

        if self._annotated_pub is not None:
            self._publish_annotated(frame, detections, header)

    def _publish_annotated(self, frame: np.ndarray, detections: List[Detection], header: Header) -> None:
        try:
            msg = self._bridge.cv2_to_imgmsg(draw_detections(frame, detections), encoding='bgr8')
        except (CvBridgeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f'[Vision] Annotated image conversion failed: {exc}', throttle_duration_sec=10.0)
            return
        msg.header = header
        self._annotated_pub.publish(msg)

    def _on_heartbeat(self) -> None:
        inferences_since_last = self._inferences - self._inferences_at_last_heartbeat
        self._inferences_at_last_heartbeat = self._inferences
        self.get_logger().info(
            f'[Vision] YOLO alive | enabled={self._enabled} | model_loaded={self._model_loaded} | '
            f'frames_received={self._frames_received} | inferences={self._inferences} | '
            f'detections={self._detections_total} | dropped_frames={self._dropped_frames} | '
            f'inference_errors={self._inference_errors} | conversion_failures={self._conversion_failures} | '
            f'inference_hz={inferences_since_last / self._heartbeat_period_sec:.1f} | '
            f'last_inference_ms={self._last_inference_ms:.0f}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = DetectorNode()
        # Two threads: one keeps swallowing camera frames while the other runs inference.
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:  # noqa: BLE001 - e.g. a parameter type error in vision.yaml
        get_logger('detector_node').fatal(f'[Vision] detector_node failed to start or crashed: {exc!r}')
        raise
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
