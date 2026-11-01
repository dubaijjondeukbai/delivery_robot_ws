"""
사람 위험 판정 히스테리시스 + 급접근(화면상 주의 신호) — ROS 의존성 없는 핵심 로직.

hazard_monitor_node 가 /vision/detections 를 받아 이 모듈에 넣는다. 단위 시험 대상이다.

용어
----
* 영상 정규화 좌표: x, y 는 영상 폭/높이로 나눈 값(0~1). 미터나 m/s 가 아니다.
* 발 위치: bbox 하단 중앙(선택: Pose 발목 키포인트 평균). 바닥 접점의 "화면상" 위치다.
* 경로 영역: 로봇 정면 통로가 화면에 보이는 사다리꼴. 미보정 초기값이며 실물에서 맞춘다.

위험 상태(HAZARD) 히스테리시스
-----------------------------
* 진입은 지연 없이: 현재 프레임의 사람 하나라도 '진입 조건'을 만족하면 그 프레임에서 바로 HAZARD.
  (연속 관측 횟수 조건은 진입에 적용하지 않는다. 새 위험의 정지를 늦추지 않기 위해서다.)
* 해제는 느리게: '해제 조건'(진입보다 완화된 영역/크기)을 만족하는 사람이 하나도 없는 프레임이
  clear_frames 번 연속, 그리고 clear_sec 이상 이어져야 CLEAR.
* 관측 누락 유예: 위험 판정을 받았던 추적 대상이 화면에서 사라지면(검출 누락/가림),
  miss_grace_sec 동안은 그 자리에 있다고 보고 해제 카운트를 시작하지 않는다.
  화면 안에서 영역 밖으로 걸어 나간 것이 관측되면 유예 없이 해제 카운트가 시작된다.
* 추적 ID 변경: ByteTrack ID 가 바뀌거나 검출 노드가 추적기를 재시작해도(전체 ID 접두어 변경)
  전역 상태는 ID 와 무관하게 유지된다. 사라진 ID 의 마지막 bbox 와 IoU 가 큰 새 ID 는
  같은 사람으로 이어 붙여(이력·유예 승계) 진단 로그에 ID_SWITCH 로 남긴다.
* 입력 단절: 검출 메시지가 input_timeout_sec 이상 없으면 STALE. stale_is_hazard 가 참이면
  출력 hazard_active 는 True (모름 = 위험으로 취급). 재개 후에도 해제 조건을 다시 채워야 CLEAR.

급접근(화면상 주의 신호, attention)
---------------------------------
같은 추적 대상의 시간 순서가 맞는 관측(헤더 stamp 증가)만으로 창(approach_window_sec) 안에서
* 경로 중심 쪽 수평 이동 속도 [영상 폭/s] — 로봇 yaw 속도로 인한 화면 이동은 공칭 HFOV 로 근사 보정,
  yaw 속도가 크면 신뢰하지 않음.
* bbox 높이의 로그 변화율 d ln(h)/dt [1/s] (확대율) — 로봇이 전진 중이면 정지 물체도 커지므로
  로봇이 거의 서 있을 때만 사용.
* 선형 회귀 결정계수(R^2), 평균 신뢰도, 관측 수, 화면 가장자리 잘림 여부
를 계산해 NONE / WATCH / ALERT 로 분류한다. 거리 정보가 없으므로 m/s 나 충돌 시간(TTC)을
계산했다고 주장하지 않는다(확대율의 역수도 TTC 로 출력하지 않는다). 이 신호는 Safety 정지
입력이 아니며, 기록·관찰용이다.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
from typing import Deque, Dict, List, Optional, Sequence, Tuple

CLEAR = 'CLEAR'
HAZARD = 'HAZARD'
STALE = 'STALE'

NONE = 'NONE'
WATCH = 'WATCH'
ALERT = 'ALERT'


# ----------------------------------------------------------------------
# 설정
# ----------------------------------------------------------------------
@dataclass
class HazardConfig:
    """vision.yaml 의 hazard_monitor_node 파라미터와 1:1 로 대응한다."""

    person_class_ids: Tuple[str, ...] = ('0', 'person')
    min_score: float = 0.40

    # 경로 영역: 정규화 (x, y) 꼭짓점 목록(시계/반시계 무관, 볼록 다각형 권장)
    path_zone: Tuple[Tuple[float, float], ...] = (
        (0.15, 1.00), (0.85, 1.00), (0.62, 0.55), (0.38, 0.55))
    # 해제용으로 영역을 바깥쪽으로 넓히는 여유(정규화 폭 단위)
    path_zone_exit_margin: float = 0.05

    # 가까움(화면상 크기) 프록시: bbox 높이 / 영상 높이
    near_height_enter: float = 0.45
    near_height_exit: float = 0.35
    # bbox 윗변이 영상 위쪽에 잘리면(가까워서 머리가 안 보임) 높이 비율이 포화되므로
    # 발 위치가 이 y 이상이면 가까운 것으로 본다.
    near_foot_y_enter: float = 0.60
    near_foot_y_exit: float = 0.57
    edge_margin: float = 0.01

    # 해제 조건
    clear_frames: int = 5
    clear_sec: float = 1.0
    # 관측 누락 유예와 추적 정리
    miss_grace_sec: float = 0.5
    track_drop_sec: float = 2.0
    # 주의 신호용 트랙 확인 관측 수 (위험 진입에는 쓰지 않는다)
    confirm_frames: int = 3
    # 입력 단절
    input_timeout_sec: float = 1.0
    stale_is_hazard: bool = True
    # ID 변경 이어 붙이기
    id_switch_iou: float = 0.3
    id_switch_max_gap_sec: float = 1.0

    # 급접근 주의 신호
    approach_window_sec: float = 1.2
    approach_min_obs: int = 4
    approach_min_span_sec: float = 0.4
    lateral_speed_watch: float = 0.15     # 영상 폭/s
    lateral_speed_alert: float = 0.30     # 영상 폭/s
    expansion_rate_watch: float = 0.25    # 1/s
    expansion_rate_alert: float = 0.50    # 1/s
    min_r2: float = 0.5
    attention_hold_sec: float = 1.0
    nominal_hfov_rad: float = math.radians(66.0)   # 미보정 공칭값(ego yaw 보정 근사용)
    max_ego_yaw_rate: float = 0.25        # rad/s, 이보다 크면 수평 이동 신호 불신
    max_ego_speed_for_expansion: float = 0.05   # m/s, 이보다 빠르면 확대율 신호 불신


# ----------------------------------------------------------------------
# 입력 단위
# ----------------------------------------------------------------------
@dataclass
class PersonObs:
    """한 프레임의 사람 검출 하나(정규화 좌표)."""

    track_id: str
    score: float
    x1: float
    y1: float
    x2: float
    y2: float
    foot: Optional[Tuple[float, float]] = None   # Pose 발목 등으로 보정한 발 위치(없으면 bbox 하단)

    @property
    def foot_point(self) -> Tuple[float, float]:
        if self.foot is not None:
            return self.foot
        return (0.5 * (self.x1 + self.x2), self.y2)

    @property
    def height(self) -> float:
        return max(0.0, self.y2 - self.y1)


@dataclass
class EgoMotion:
    """로봇 자신의 움직임(차체 기준). known=False 면 모름."""

    v: float = 0.0
    w: float = 0.0
    known: bool = False


# ----------------------------------------------------------------------
# 기하 함수
# ----------------------------------------------------------------------
def point_in_polygon(x: float, y: float, polygon: Sequence[Tuple[float, float]]) -> bool:
    """짝수-홀수 규칙. 경계 위는 안쪽으로 취급한다."""
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            t = (y - y1) / (y2 - y1)
            xc = x1 + t * (x2 - x1)
            if abs(xc - x) < 1e-12:
                return True
            if xc > x:
                inside = not inside
    return inside


def expand_polygon(polygon: Sequence[Tuple[float, float]], margin: float
                   ) -> List[Tuple[float, float]]:
    """무게중심 기준으로 꼭짓점을 margin 만큼 바깥으로 민다(해제용 완화 영역)."""
    cx = sum(p[0] for p in polygon) / len(polygon)
    cy = sum(p[1] for p in polygon) / len(polygon)
    out = []
    for x, y in polygon:
        dx, dy = x - cx, y - cy
        norm = math.hypot(dx, dy) or 1.0
        out.append((x + margin * dx / norm, y + margin * dy / norm))
    return out


def iou(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]) -> float:
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0.0 else 0.0


def linear_fit(ts: Sequence[float], ys: Sequence[float]) -> Tuple[float, float]:
    """(기울기, R^2). 점이 2개 미만이거나 시간 폭이 0 이면 (0, 0)."""
    n = len(ts)
    if n < 2:
        return 0.0, 0.0
    mt = sum(ts) / n
    my = sum(ys) / n
    stt = sum((t - mt) ** 2 for t in ts)
    if stt <= 1e-12:
        return 0.0, 0.0
    sty = sum((t - mt) * (y - my) for t, y in zip(ts, ys))
    slope = sty / stt
    syy = sum((y - my) ** 2 for y in ys)
    if syy <= 1e-12:
        return slope, 1.0
    r2 = (sty * sty) / (stt * syy)
    return slope, max(0.0, min(1.0, r2))


# ----------------------------------------------------------------------
# 추적 대상 상태
# ----------------------------------------------------------------------
@dataclass
class TrackState:
    track_id: str
    first_seen: float
    last_seen: float
    observations: int = 0
    last_box: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    last_score: float = 0.0
    hazard_hold: bool = False        # 마지막 관측에서 해제 조건(완화)을 만족했는가
    history: Deque[Tuple[float, float, float, float, float, float, bool]] = field(
        default_factory=lambda: deque(maxlen=64))
    # history 항목: (stamp, foot_x, foot_y, height, score, ego_w, ego_known)
    attention: str = NONE
    attention_until: float = -1.0
    attention_info: Dict[str, object] = field(default_factory=dict)
    aliases: List[str] = field(default_factory=list)


# ----------------------------------------------------------------------
# 필터
# ----------------------------------------------------------------------
class PersonHazardFilter:
    """
    update(stamp, people, ego) 를 프레임마다, tick(now) 를 주기적으로 호출한다.

    stamp: 영상 header.stamp(초). now: 입력 단절 판단용 단조 시계(초).
    두 시계를 섞지 않는다: 이력/속도는 stamp, 단절/유예는 stamp 기준 경과로 계산하고,
    메시지가 아예 안 오는 경우만 now 로 판단한다.
    """

    def __init__(self, config: HazardConfig):
        self.c = config
        self._zone = list(config.path_zone)
        self._zone_exit = expand_polygon(self._zone, config.path_zone_exit_margin)
        self.state = STALE           # 첫 메시지 전에는 모름
        self.reason = 'waiting_for_detections'
        self.tracks: Dict[str, TrackState] = {}
        self._clear_frames = 0
        self._clear_since: Optional[float] = None
        self._last_stamp: Optional[float] = None
        self._last_input_now: Optional[float] = None
        self._raw_prev: Optional[bool] = None
        self.events: List[Dict[str, object]] = []
        self.counters: Dict[str, int] = {
            'frames': 0, 'raw_toggles': 0, 'state_changes': 0, 'hazard_entries': 0,
            'id_switches': 0, 'out_of_order': 0, 'stale': 0, 'alerts': 0,
        }

    # ------------------------------------------------------------------
    @property
    def hazard_active(self) -> bool:
        """Safety 등 소비자에게 줄 값: HAZARD, 또는 STALE 이면서 stale_is_hazard."""
        return self.state == HAZARD or (self.state == STALE and self.c.stale_is_hazard)

    def _emit(self, kind: str, **data):
        record = {'event': kind, **data}
        self.events.append(record)
        if len(self.events) > 500:     # 소비자가 drain_events() 를 안 부를 때의 상한
            del self.events[:250]

    def drain_events(self) -> List[Dict[str, object]]:
        """쌓인 진단 이벤트를 꺼내고 비운다."""
        events, self.events = self.events, []
        return events

    def _set_state(self, state: str, reason: str, stamp: Optional[float]):
        if state != self.state:
            self.counters['state_changes'] += 1
            if state == HAZARD:
                self.counters['hazard_entries'] += 1
            self._emit('STATE', old=self.state, new=state, reason=reason, stamp=stamp)
        self.state = state
        self.reason = reason

    # ------------------------------------------------------------------
    def _is_person(self, class_id: str) -> bool:
        return class_id in self.c.person_class_ids

    def classify(self, obs: PersonObs) -> Tuple[bool, bool, str]:
        """(진입 조건, 해제 유지 조건, 사유 문자열)."""
        fx, fy = obs.foot_point
        top_cut = obs.y1 <= self.c.edge_margin
        in_zone = point_in_polygon(fx, fy, self._zone)
        in_exit_zone = point_in_polygon(fx, fy, self._zone_exit)
        near_enter = obs.height >= self.c.near_height_enter or (
            top_cut and fy >= self.c.near_foot_y_enter)
        near_exit = obs.height >= self.c.near_height_exit or (
            top_cut and fy >= self.c.near_foot_y_exit)
        enter = obs.score >= self.c.min_score and in_zone and near_enter
        hold = obs.score >= self.c.min_score and in_exit_zone and near_exit
        reason = (f'id={obs.track_id} score={obs.score:.2f} foot=({fx:.2f},{fy:.2f}) '
                  f'h={obs.height:.2f} in_zone={in_zone} top_cut={top_cut}')
        return enter, hold, reason

    # ------------------------------------------------------------------
    def _match_id_switch(self, obs: PersonObs, stamp: float, seen_ids: set) -> Optional[str]:
        """새 ID 가 최근 사라진 트랙과 겹치면 그 트랙 ID 를 돌려준다."""
        best, best_iou = None, self.c.id_switch_iou
        box = (obs.x1, obs.y1, obs.x2, obs.y2)
        for tid, tr in self.tracks.items():
            if tid in seen_ids:
                continue
            gap = stamp - tr.last_seen
            if gap <= 0.0 or gap > self.c.id_switch_max_gap_sec:
                continue
            value = iou(box, tr.last_box)
            if value >= best_iou:
                best, best_iou = tid, value
        return best

    def update(self, stamp: float, people: Sequence[PersonObs], ego: EgoMotion,
               now: float) -> None:
        """한 프레임 처리. stamp 가 이전 프레임보다 이르면 무시한다(시간 순서 보장)."""
        if self._last_stamp is not None and stamp <= self._last_stamp:
            self.counters['out_of_order'] += 1
            self._emit('OUT_OF_ORDER', stamp=stamp, last=self._last_stamp)
            return
        self._last_stamp = stamp
        self._last_input_now = now
        self.counters['frames'] += 1

        any_enter, any_hold = False, False
        enter_reason = ''
        # 이번 프레임에 이미 있는 ID 는 ID 변경 이어 붙이기 후보에서 뺀다(순서와 무관하게).
        seen_ids = {obs.track_id for obs in people if obs.track_id}

        for obs in people:
            tid = obs.track_id or ''
            if not tid or tid not in self.tracks:
                previous = self._match_id_switch(obs, stamp, seen_ids - {tid})
                if previous is not None:
                    if not tid:
                        tid = previous
                    else:
                        moved = self.tracks.pop(previous)
                        moved.aliases.append(previous)
                        moved.track_id = tid
                        self.tracks[tid] = moved
                        self.counters['id_switches'] += 1
                        self._emit('ID_SWITCH', old=previous, new=tid, stamp=stamp)
                elif not tid:
                    tid = f'anon-{self.counters["frames"]}-{len(seen_ids)}'
                obs.track_id = tid
            seen_ids.add(tid)

            tr = self.tracks.get(tid)
            if tr is None:
                tr = TrackState(track_id=tid, first_seen=stamp, last_seen=stamp)
                self.tracks[tid] = tr

            enter, hold, reason = self.classify(obs)
            tr.last_seen = stamp
            tr.observations += 1
            tr.last_box = (obs.x1, obs.y1, obs.x2, obs.y2)
            tr.last_score = obs.score
            tr.hazard_hold = hold
            fx, fy = obs.foot_point
            tr.history.append((stamp, fx, fy, max(obs.height, 1e-6), obs.score, ego.w, ego.known))
            if enter and not any_enter:
                enter_reason = reason
            any_enter = any_enter or enter
            any_hold = any_hold or hold
            self._update_attention(tr, stamp, ego, obs)

        # 관측 누락 유예: 위험 유지 상태로 사라진 트랙은 잠시 그대로 있다고 본다.
        coasting = [tid for tid, tr in self.tracks.items()
                    if tid not in seen_ids and tr.hazard_hold
                    and stamp - tr.last_seen <= self.c.miss_grace_sec]

        # 오래된 트랙 정리
        for tid in [tid for tid, tr in self.tracks.items()
                    if stamp - tr.last_seen > self.c.track_drop_sec]:
            del self.tracks[tid]

        raw = any_enter
        if self._raw_prev is not None and raw != self._raw_prev:
            self.counters['raw_toggles'] += 1
        self._raw_prev = raw

        if any_enter:
            self._clear_frames = 0
            self._clear_since = None
            self._set_state(HAZARD, 'enter: ' + enter_reason, stamp)
            return

        if self.state == CLEAR:
            return

        # HAZARD 또는 STALE 에서의 해제 판단
        if any_hold or coasting:
            self._clear_frames = 0
            self._clear_since = None
            why = 'hold: person still near/in zone' if any_hold else (
                'hold: missing-detection grace ' + ','.join(coasting))
            if self.state == STALE:
                # 입력이 돌아왔고 아직 위험 유지 조건이므로 HAZARD 로 표시
                self._set_state(HAZARD, why, stamp)
            else:
                self.reason = why
            return

        self._clear_frames += 1
        if self._clear_since is None:
            self._clear_since = stamp
        if (self._clear_frames >= self.c.clear_frames
                and stamp - self._clear_since >= self.c.clear_sec):
            self._set_state(CLEAR, f'released after {self._clear_frames} clear frames', stamp)
            self._clear_frames = 0
            self._clear_since = None
        else:
            self.reason = (f'releasing: {self._clear_frames}/{self.c.clear_frames} frames, '
                           f'{stamp - self._clear_since:.2f}/{self.c.clear_sec:.2f}s')

    def tick(self, now: float) -> None:
        """입력 단절 감시(단조 시계)."""
        if self._last_input_now is None:
            return
        if now - self._last_input_now > self.c.input_timeout_sec and self.state != STALE:
            self.counters['stale'] += 1
            self._clear_frames = 0
            self._clear_since = None
            self._set_state(STALE, f'no detections for {now - self._last_input_now:.2f}s',
                            self._last_stamp)

    # ------------------------------------------------------------------
    def _update_attention(self, tr: TrackState, stamp: float, ego: EgoMotion,
                          obs: PersonObs) -> None:
        c = self.c
        window = [h for h in tr.history if stamp - h[0] <= c.approach_window_sec]
        info: Dict[str, object] = {'obs': len(window)}
        level = NONE
        reasons: List[str] = []

        if tr.observations < c.confirm_frames:
            reasons.append('unconfirmed_track')
        elif len(window) < c.approach_min_obs or window[-1][0] - window[0][0] < (
                c.approach_min_span_sec):
            reasons.append('short_history')
        else:
            ts = [h[0] - window[0][0] for h in window]
            fx = [h[1] for h in window]
            logh = [math.log(h[3]) for h in window]
            vx, r2x = linear_fit(ts, fx)
            exp_rate, r2h = linear_fit(ts, logh)
            mean_w = sum(h[5] for h in window) / len(window)
            ego_known = all(h[6] for h in window)
            mean_score = sum(h[4] for h in window) / len(window)

            # 로봇 yaw 에 의한 화면 이동(공칭 HFOV 근사): 반시계(+w) 회전 → 정지 물체가 오른쪽(+x)으로
            vx_comp = vx - (mean_w / c.nominal_hfov_rad if ego_known else 0.0)
            # 경로 쪽 수평 속도: 영역 밖이면 영역 중심 방향 성분, 영역 안이면 가로지르는 속도 크기
            center_x = sum(p[0] for p in self._zone) / len(self._zone)
            fx_now, fy_now = fx[-1], window[-1][2]
            if point_in_polygon(fx_now, fy_now, self._zone):
                toward = abs(vx_comp)
            else:
                toward = -math.copysign(1.0, fx_now - center_x) * vx_comp

            lateral_ok = ego_known and abs(mean_w) <= c.max_ego_yaw_rate and r2x >= c.min_r2
            expansion_ok = (ego_known and abs(ego.v) <= c.max_ego_speed_for_expansion
                            and r2h >= c.min_r2)
            edge_cut = obs.x1 <= c.edge_margin or obs.x2 >= 1.0 - c.edge_margin
            if not ego_known:
                reasons.append('ego_motion_unknown')
            elif abs(mean_w) > c.max_ego_yaw_rate:
                reasons.append('ego_rotation')
            if ego_known and abs(ego.v) > c.max_ego_speed_for_expansion:
                reasons.append('ego_forward_motion')
            if edge_cut:
                reasons.append('bbox_cut_at_image_side')
            if mean_score < c.min_score:
                reasons.append('low_score')

            info.update({
                'lateral_speed_toward_path_w_per_s': round(toward, 3),
                'lateral_r2': round(r2x, 2),
                'expansion_rate_per_s': round(exp_rate, 3),
                'expansion_r2': round(r2h, 2),
                'mean_score': round(mean_score, 2),
                'lateral_reliable': lateral_ok and not edge_cut,
                'expansion_reliable': expansion_ok,
            })

            usable = mean_score >= c.min_score
            if usable and lateral_ok and not edge_cut:
                if toward >= c.lateral_speed_alert:
                    level = ALERT
                    reasons.append('fast_lateral_toward_path')
                elif toward >= c.lateral_speed_watch:
                    level = WATCH
                    reasons.append('lateral_toward_path')
            if usable and expansion_ok:
                if exp_rate >= c.expansion_rate_alert:
                    level = ALERT
                    reasons.append('rapid_expansion')
                elif exp_rate >= c.expansion_rate_watch and level == NONE:
                    level = WATCH
                    reasons.append('expansion')

        if level == ALERT:
            if tr.attention != ALERT:
                self.counters['alerts'] += 1
                self._emit('ATTENTION', id=tr.track_id, level=level, reasons=reasons,
                           stamp=stamp, **{k: v for k, v in info.items() if k != 'obs'})
            tr.attention_until = stamp + c.attention_hold_sec
            tr.attention = ALERT
        elif tr.attention == ALERT and stamp < tr.attention_until:
            reasons.append('alert_hold')
        else:
            tr.attention = level
        info['reasons'] = reasons
        tr.attention_info = info

    # ------------------------------------------------------------------
    def attention_level(self) -> str:
        levels = [tr.attention for tr in self.tracks.values()]
        if ALERT in levels:
            return ALERT
        if WATCH in levels:
            return WATCH
        return NONE

    def snapshot(self) -> Dict[str, object]:
        return {
            'state': self.state,
            'hazard_active': self.hazard_active,
            'reason': self.reason,
            'attention': self.attention_level(),
            'attention_is_image_space_only': True,
            'tracks': [
                {'id': tr.track_id, 'obs': tr.observations, 'score': round(tr.last_score, 2),
                 'box': [round(v, 3) for v in tr.last_box], 'hazard_hold': tr.hazard_hold,
                 'attention': tr.attention, 'cues': tr.attention_info,
                 'aliases': tr.aliases[-3:]}
                for tr in self.tracks.values()],
            'counters': dict(self.counters),
        }
