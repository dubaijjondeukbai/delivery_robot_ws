#!/usr/bin/env python3
"""
[Vision] hazard_monitor_node — 사람 위험 판정 히스테리시스 + 급접근 주의 신호.

입력
----
/vision/detections   vision_msgs/Detection2DArray  (detector_node, 사람 클래스만 사용)
/camera/camera_info  sensor_msgs/CameraInfo         (영상 크기만 사용; 없으면 파라미터 값)
/odom                nav_msgs/Odometry              (로봇 자신의 움직임, 없으면 /cmd_vel_safe)
/cmd_vel_safe        geometry_msgs/Twist            (odom 이 없을 때 대체)
/vision/poses        std_msgs/String(JSON)          (선택, use_pose_keypoints=true 일 때 발 위치 보정)

출력
----
/vision/person_hazard  std_msgs/Bool    HAZARD 또는 (STALE 이고 stale_is_hazard) 이면 True
/vision/hazard_state   std_msgs/String  JSON: 상태, 사유, 트랙별 화면상 단서, 카운터

* 진입은 지연 없음, 해제는 clear_frames + clear_sec, 누락 유예, ID 변경 이어 붙이기
  (자세한 규칙은 robot_vision/person_hazard.py 문서).
* 급접근 신호(attention)는 화면 좌표 기반 주의 신호다. 거리·m/s·충돌 시간을 계산하지 않는다.
* 이 노드는 모터/Safety 를 직접 제어하지 않는다. robot_safety 는 use_vision_hazard:=true 일 때만
  /vision/person_hazard 를 정지 입력으로 쓴다(기본 false).
"""

from __future__ import annotations

import json
import math
import time
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from robot_vision.person_hazard import (
    ALERT,
    EgoMotion,
    HAZARD,
    HazardConfig,
    iou,
    PersonHazardFilter,
    PersonObs,
    STALE,
)
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray

ANKLES = (15, 16)    # COCO keypoint index: left/right ankle


def _detection_fields(det) -> Tuple[str, float, float, float, float, float, str]:
    """Detection2D -> (class_id, score, cx, cy, w, h, track_id). vision_msgs 4.0/4.1 모두 지원."""
    center = det.bbox.center
    if hasattr(center, 'position'):
        cx, cy = center.position.x, center.position.y
    else:
        cx, cy = center.x, center.y
    class_id, score = '', 0.0
    if det.results:
        hyp = det.results[0]
        if hasattr(hyp, 'hypothesis'):
            class_id, score = str(hyp.hypothesis.class_id), float(hyp.hypothesis.score)
        else:
            class_id, score = str(getattr(hyp, 'id', '')), float(getattr(hyp, 'score', 0.0))
    track_id = str(getattr(det, 'id', '') or '')
    return class_id, score, float(cx), float(cy), float(det.bbox.size_x), float(
        det.bbox.size_y), track_id


class HazardMonitorNode(Node):
    """사람 위험/주의 상태를 안정화해 발행한다."""

    def __init__(self):
        super().__init__('hazard_monitor_node')
        c = HazardConfig()
        defaults = {
            'enabled': True,
            'detections_topic': '/vision/detections',
            'camera_info_topic': '/camera/camera_info',
            'poses_topic': '/vision/poses',
            'use_pose_keypoints': False,
            'ego_odom_topic': '/odom',
            'ego_cmd_topic': '/cmd_vel_safe',
            'ego_timeout_sec': 0.5,
            'hazard_topic': '/vision/person_hazard',
            'state_topic': '/vision/hazard_state',
            'publish_rate_hz': 10.0,
            'heartbeat_period_sec': 5.0,
            'image_width': 640,
            'image_height': 480,
            'person_class_ids': list(c.person_class_ids),
            'min_score': c.min_score,
            'path_zone': [v for xy in c.path_zone for v in xy],
            'path_zone_exit_margin': c.path_zone_exit_margin,
            'near_height_enter': c.near_height_enter,
            'near_height_exit': c.near_height_exit,
            'near_foot_y_enter': c.near_foot_y_enter,
            'near_foot_y_exit': c.near_foot_y_exit,
            'clear_frames': c.clear_frames,
            'clear_sec': c.clear_sec,
            'miss_grace_sec': c.miss_grace_sec,
            'track_drop_sec': c.track_drop_sec,
            'confirm_frames': c.confirm_frames,
            'input_timeout_sec': c.input_timeout_sec,
            'stale_is_hazard': c.stale_is_hazard,
            'id_switch_iou': c.id_switch_iou,
            'id_switch_max_gap_sec': c.id_switch_max_gap_sec,
            'approach_window_sec': c.approach_window_sec,
            'approach_min_obs': c.approach_min_obs,
            'approach_min_span_sec': c.approach_min_span_sec,
            'lateral_speed_watch': c.lateral_speed_watch,
            'lateral_speed_alert': c.lateral_speed_alert,
            'expansion_rate_watch': c.expansion_rate_watch,
            'expansion_rate_alert': c.expansion_rate_alert,
            'min_r2': c.min_r2,
            'attention_hold_sec': c.attention_hold_sec,
            'nominal_hfov_deg': math.degrees(c.nominal_hfov_rad),
            'max_ego_yaw_rate': c.max_ego_yaw_rate,
            'max_ego_speed_for_expansion': c.max_ego_speed_for_expansion,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in defaults}
        self.p = p
        self._enabled = bool(p['enabled'])

        zone = list(p['path_zone'])
        if len(zone) < 6 or len(zone) % 2:
            raise ValueError('path_zone must be a flat list x1,y1,x2,y2,... with >= 3 points')
        if not all(0.0 <= float(v) <= 1.0 for v in zone):
            raise ValueError('path_zone values must be normalized to [0, 1]')
        if int(p['clear_frames']) < 1 or float(p['clear_sec']) < 0.0:
            raise ValueError('clear_frames >= 1 and clear_sec >= 0 required')
        if float(p['near_height_exit']) > float(p['near_height_enter']):
            raise ValueError('near_height_exit must be <= near_height_enter (hysteresis)')

        cfg = HazardConfig(
            person_class_ids=tuple(str(v) for v in p['person_class_ids']),
            min_score=float(p['min_score']),
            path_zone=tuple((float(zone[i]), float(zone[i + 1]))
                            for i in range(0, len(zone), 2)),
            path_zone_exit_margin=float(p['path_zone_exit_margin']),
            near_height_enter=float(p['near_height_enter']),
            near_height_exit=float(p['near_height_exit']),
            near_foot_y_enter=float(p['near_foot_y_enter']),
            near_foot_y_exit=float(p['near_foot_y_exit']),
            clear_frames=int(p['clear_frames']),
            clear_sec=float(p['clear_sec']),
            miss_grace_sec=float(p['miss_grace_sec']),
            track_drop_sec=float(p['track_drop_sec']),
            confirm_frames=int(p['confirm_frames']),
            input_timeout_sec=float(p['input_timeout_sec']),
            stale_is_hazard=bool(p['stale_is_hazard']),
            id_switch_iou=float(p['id_switch_iou']),
            id_switch_max_gap_sec=float(p['id_switch_max_gap_sec']),
            approach_window_sec=float(p['approach_window_sec']),
            approach_min_obs=int(p['approach_min_obs']),
            approach_min_span_sec=float(p['approach_min_span_sec']),
            lateral_speed_watch=float(p['lateral_speed_watch']),
            lateral_speed_alert=float(p['lateral_speed_alert']),
            expansion_rate_watch=float(p['expansion_rate_watch']),
            expansion_rate_alert=float(p['expansion_rate_alert']),
            min_r2=float(p['min_r2']),
            attention_hold_sec=float(p['attention_hold_sec']),
            nominal_hfov_rad=math.radians(float(p['nominal_hfov_deg'])),
            max_ego_yaw_rate=float(p['max_ego_yaw_rate']),
            max_ego_speed_for_expansion=float(p['max_ego_speed_for_expansion']),
        )
        self._filter = PersonHazardFilter(cfg)
        self._image_size: Tuple[int, int] = (int(p['image_width']), int(p['image_height']))
        self._image_size_from_info = False
        self._ego_odom: Optional[Tuple[float, float, float]] = None   # v, w, monotonic t
        self._ego_cmd: Optional[Tuple[float, float, float]] = None
        self._poses: Dict[int, List[dict]] = {}

        reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        input_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                               reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)

        self._hazard_pub = self.create_publisher(Bool, p['hazard_topic'], reliable)
        self._state_pub = self.create_publisher(String, p['state_topic'], reliable)
        self._timer = self.create_timer(1.0 / float(p['publish_rate_hz']), self._on_timer)
        self._heartbeat = self.create_timer(float(p['heartbeat_period_sec']),
                                            self._on_heartbeat)

        if not self._enabled:
            self.get_logger().warning('[Hazard] disabled: publishes nothing')
            self._timer.cancel()
            return

        self.create_subscription(Detection2DArray, p['detections_topic'],
                                 self._on_detections, input_qos)
        self.create_subscription(CameraInfo, p['camera_info_topic'], self._on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, p['ego_odom_topic'], self._on_odom, input_qos)
        self.create_subscription(Twist, p['ego_cmd_topic'], self._on_cmd, input_qos)
        if p['use_pose_keypoints']:
            self.create_subscription(String, p['poses_topic'], self._on_poses, input_qos)

        detections_topic, hazard_topic = p['detections_topic'], p['hazard_topic']
        self.get_logger().info(
            f'[Hazard] ready: input={detections_topic} output={hazard_topic} '
            f'clear={cfg.clear_frames} frames/{cfg.clear_sec:.1f}s '
            f'grace={cfg.miss_grace_sec:.1f}s stale_is_hazard={cfg.stale_is_hazard} '
            '(image-space attention only; no metric distance)')

    # ------------------------------------------------------------------
    def _on_info(self, msg: CameraInfo):
        if msg.width > 0 and msg.height > 0:
            self._image_size = (int(msg.width), int(msg.height))
            self._image_size_from_info = True

    def _on_odom(self, msg: Odometry):
        self._ego_odom = (msg.twist.twist.linear.x, msg.twist.twist.angular.z, time.monotonic())

    def _on_cmd(self, msg: Twist):
        self._ego_cmd = (msg.linear.x, msg.angular.z, time.monotonic())

    def _ego(self) -> EgoMotion:
        now = time.monotonic()
        timeout = float(self.p['ego_timeout_sec'])
        for source in (self._ego_odom, self._ego_cmd):
            if source is not None and now - source[2] <= timeout:
                return EgoMotion(v=float(source[0]), w=float(source[1]), known=True)
        return EgoMotion(known=False)

    def _on_poses(self, msg: String):
        try:
            data = json.loads(msg.data)
            key = int(round(float(data['stamp']) * 1000.0))
            width, height = float(data['width']), float(data['height'])
            people = []
            for person in data.get('people', []):
                x1, y1, x2, y2 = (float(v) for v in person['bbox'])
                people.append({'box': (x1 / width, y1 / height, x2 / width, y2 / height),
                               'keypoints': person.get('keypoints') or [],
                               'width': width, 'height': height})
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f'[Hazard] bad pose JSON: {exc!r}',
                                      throttle_duration_sec=5.0)
            return
        self._poses[key] = people
        for old in sorted(self._poses)[:-20]:
            del self._poses[old]

    def _pose_foot(self, stamp_ms: int, box) -> Optional[Tuple[float, float]]:
        """같은 영상(stamp)의 Pose 결과 중 IoU 가 큰 사람의 발목 평균. 보조 정보일 뿐이다."""
        people = self._poses.get(stamp_ms)
        if not people:
            return None
        best = max(people, key=lambda person: iou(box, person['box']))
        if iou(box, best['box']) < 0.5:
            return None
        points = []
        for index in ANKLES:
            if index < len(best['keypoints']):
                x, y, conf = best['keypoints'][index]
                if conf >= 0.5:
                    points.append((x / best['width'], y / best['height']))
        if not points:
            return None
        return (sum(pt[0] for pt in points) / len(points),
                sum(pt[1] for pt in points) / len(points))

    def _on_detections(self, msg: Detection2DArray):
        width, height = self._image_size
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        stamp_ms = int(round(stamp * 1000.0))
        people: List[PersonObs] = []
        for det in msg.detections:
            class_id, score, cx, cy, w, h, track_id = _detection_fields(det)
            if class_id not in self._filter.c.person_class_ids:
                continue
            box = ((cx - w / 2) / width, (cy - h / 2) / height,
                   (cx + w / 2) / width, (cy + h / 2) / height)
            foot = self._pose_foot(stamp_ms, box) if self.p['use_pose_keypoints'] else None
            people.append(PersonObs(track_id, score, *box, foot=foot))
        self._filter.update(stamp, people, self._ego(), time.monotonic())
        self._flush_events()
        self._publish()

    def _on_timer(self):
        self._filter.tick(time.monotonic())
        self._flush_events()
        self._publish()

    def _publish(self):
        self._hazard_pub.publish(Bool(data=bool(self._filter.hazard_active)))
        snapshot = self._filter.snapshot()
        snapshot['image_size'] = list(self._image_size)
        snapshot['image_size_from_camera_info'] = self._image_size_from_info
        ego = self._ego()
        snapshot['ego'] = {'known': ego.known, 'v': round(ego.v, 3), 'w': round(ego.w, 3)}
        self._state_pub.publish(String(data=json.dumps(snapshot, ensure_ascii=False)))

    def _flush_events(self):
        for event in self._filter.drain_events():
            kind = event['event']
            text = json.dumps(event, ensure_ascii=False)
            if kind == 'STATE' and event['new'] in (HAZARD, STALE):
                self.get_logger().warning(f'[Hazard] {text}')
            elif kind == 'ATTENTION' and event.get('level') == ALERT:
                self.get_logger().warning(f'[Hazard] {text}')
            elif kind == 'OUT_OF_ORDER':
                self.get_logger().warning(f'[Hazard] {text}', throttle_duration_sec=5.0)
            else:
                self.get_logger().info(f'[Hazard] {text}')

    def _on_heartbeat(self):
        f = self._filter
        self.get_logger().info(
            f'[Hazard] alive state={f.state} active={f.hazard_active} '
            f'attention={f.attention_level()} tracks={len(f.tracks)} '
            f'counters={json.dumps(f.counters)} reason="{f.reason}"')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HazardMonitorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
