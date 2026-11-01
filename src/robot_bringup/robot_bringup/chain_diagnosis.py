"""
명령 흐름 진단용 순수 함수 모음 (ROS 의존성 없음, 단위 시험 대상).

motion_chain_monitor 노드가 토픽 값을 모아 Snapshot 을 만들고,
여기의 함수가 "어느 단계에서 명령이 끊겼는지"와 "왜 멈췄는지"를 분류한다.

    Nav2 controller/behavior -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_auto
      -> safety_manager -> /cmd_vel_safe -> (Gazebo DiffDrive | 실물 bridge)
      -> /wheel/odom -> EKF -> /odom

판단 원칙
---------
* 값이 0 인 명령과 "메시지가 오지 않음"을 구분한다 (age 로 판단).
* 정지 사유는 /safety/state 문자열을 그대로 분류한다(추측하지 않는다).
* 회전 비교는 실제 회전 명령이 나간 구간(|/cmd_vel_safe.angular.z| 가 임계값 이상)만
  적분한다. 정지 상태에서 각속도 0 끼리 비교한 결과로 EKF 를 판정하지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import re
from typing import Dict, List, Optional, Tuple


# ----------------------------------------------------------------------
# /safety/state 분류
# ----------------------------------------------------------------------
SAFETY_CATEGORIES = {
    'waiting_for_estop_state': 'estop',
    'estop_state_timeout': 'estop',
    'estop_active': 'estop',
    'waiting_for_obstacle_state': 'obstacle',
    'obstacle_state_timeout': 'obstacle',
    'obstacle_detected': 'obstacle',
    'vision_hazard': 'vision',
    'vision_hazard_timeout': 'vision',
    'waiting_for_vision_hazard': 'vision',
    'command_timeout': 'command',
    'waiting_for_new_command': 'command',
    'invalid_command_nonfinite': 'command',
    'unsupported_command_axes': 'command',
    'shutdown': 'command',
}


def safety_reason(state: Optional[str]) -> str:
    """'READY' -> '', 'STOP:<사유>' -> '<사유>', 그 밖은 'unknown_state'."""
    if state is None:
        return 'no_state'
    if state == 'READY':
        return ''
    if state.startswith('STOP:'):
        return state[5:]
    return 'unknown_state'


def safety_category(state: Optional[str]) -> str:
    """정지 사유를 estop / obstacle / vision / command / none / unknown 으로 묶는다."""
    reason = safety_reason(state)
    if reason == '':
        return 'none'
    if reason == 'no_state':
        return 'unknown'
    return SAFETY_CATEGORIES.get(reason, 'unknown')


# ----------------------------------------------------------------------
# /rosout 메시지 분류 (Nav2 / AMCL / Safety)
# ----------------------------------------------------------------------
ROSOUT_PATTERNS: List[Tuple[str, re.Pattern]] = [
    # FollowPath INVALID_PATH(103): 경로 자체가 로컬 창으로 변환되지 않음.
    ('INVALID_PATH_0_POSES', re.compile(r'Resulting plan has 0 poses')),
    # FollowPath FAILED_TO_MAKE_PROGRESS(105): 경로는 있지만 로봇이 움직이지 않음.
    ('FAILED_TO_MAKE_PROGRESS', re.compile(r'Failed to make progress')),
    ('GOAL_SUCCEEDED', re.compile(r'Goal succeeded|Reached the goal')),
    ('GOAL_FAILED', re.compile(r'Goal failed|Aborting handle|aborted', re.I)),
    ('GOAL_CANCELED', re.compile(r'Goal canceled|Goal was canceled|cancel', re.I)),
    ('RECOVERY_TIMEOUT', re.compile(r'[Ee]xceeded time allowance|timed out|time allowance')),
    ('COLLISION_AHEAD', re.compile(r'[Cc]ollision [Aa]head|[Dd]etected collision')),
    ('BEHAVIOR_RUNNING', re.compile(r'Running (spin|backup|wait|drive_on_heading)', re.I)),
    ('AMCL_REINIT', re.compile(r'initialPoseReceived|Setting pose')),
    ('TF_ERROR', re.compile(r'[Ee]xtrapolation|[Ll]ookup would require|'
                            r'Could not transform|Timed out waiting for transform')),
    ('NO_VALID_CONTROL', re.compile(r'[Nn]o valid control|NO_VALID_CONTROL')),
    # BT 가 서버 응답을 default_server_timeout 안에 못 받음(부하가 큰 PC/Pi 에서 발생)
    ('BT_SERVER_TIMEOUT', re.compile(r'Timed out while waiting for action server')),
    ('PLANNER_FAILED', re.compile(r'[Ff]ailed to create plan|GridBased plugin failed|'
                                  r'[Nn]o valid path|Planning algorithm failed')),
]


LIDAR_STOP = re.compile(r'\[LiDAR\] STOP: (\w+)')


def lidar_stop_reason(text: str) -> Optional[str]:
    """
    lidar_obstacle_node 의 정지 세부 사유.

    /safety/state 는 모두 'obstacle_detected' 로 보이지만 실제로는 장애물(obstacle_in_stop_zone)
    과 데이터 신선도(scan_timeout, stale_scan_stamp 등)가 섞여 있다.
    """
    match = LIDAR_STOP.search(text)
    return match.group(1) if match else None


def classify_rosout(text: str) -> Optional[str]:
    """첫 번째로 일치하는 이벤트 이름, 없으면 None."""
    for name, pattern in ROSOUT_PATTERNS:
        if pattern.search(text):
            return name
    return None


# ----------------------------------------------------------------------
# 스냅샷과 단계별 끊김 판단
# ----------------------------------------------------------------------
@dataclass
class TwistSample:
    """선속도/각속도와 마지막 수신 후 경과 시간(초). age=None 은 한 번도 못 받음."""

    v: float = 0.0
    w: float = 0.0
    age: Optional[float] = None

    def fresh(self, max_age: float) -> bool:
        return self.age is not None and self.age <= max_age

    def moving(self, v_eps: float, w_eps: float) -> bool:
        return abs(self.v) > v_eps or abs(self.w) > w_eps


@dataclass
class Snapshot:
    """한 시점의 명령 흐름 상태."""

    nav: TwistSample = field(default_factory=TwistSample)
    dry_run: TwistSample = field(default_factory=TwistSample)
    auto: TwistSample = field(default_factory=TwistSample)
    safe: TwistSample = field(default_factory=TwistSample)
    wheel: TwistSample = field(default_factory=TwistSample)
    odom: TwistSample = field(default_factory=TwistSample)
    imu_wz: Optional[float] = None
    imu_age: Optional[float] = None
    safety_state: Optional[str] = None
    safety_age: Optional[float] = None
    estop: Optional[bool] = None
    estop_age: Optional[float] = None
    obstacle: Optional[bool] = None
    obstacle_age: Optional[float] = None
    goal_active: bool = False
    tf_map_odom_ok: Optional[bool] = None
    tf_odom_base_ok: Optional[bool] = None


@dataclass
class Thresholds:
    cmd_fresh_sec: float = 0.5
    state_fresh_sec: float = 1.0
    sensor_fresh_sec: float = 0.5
    v_eps: float = 0.02
    w_eps: float = 0.05


def diagnose(s: Snapshot, th: Thresholds = Thresholds()) -> List[str]:
    """
    끊긴 단계를 위에서 아래 순서로 찾는다. 첫 항목이 가장 상류의 원인이다.

    반환값은 사람이 읽는 짧은 문장 목록이다(같은 문장이 반복되면 노드가 이벤트로 기록).
    """
    findings: List[str] = []

    if s.safety_age is None or s.safety_age > th.state_fresh_sec:
        findings.append('safety_manager 출력(/safety/state) 없음: 노드 미실행 또는 QoS/도메인 불일치')

    # 1) Nav2 출력 단계
    if s.goal_active and not s.nav.fresh(th.cmd_fresh_sec):
        if s.dry_run.fresh(th.cmd_fresh_sec):
            findings.append('Nav2 출력이 /cmd_vel_nav_dry_run 으로 감: motion_enabled:=false 로 실행됨')
        else:
            findings.append('목표 실행 중인데 /cmd_vel_nav 가 없음: controller/behavior 가 명령을 안 냄'
                            ' (TF·코스트맵·경로 실패 로그 확인)')

    # 2) velocity_smoother 단계
    if s.nav.fresh(th.cmd_fresh_sec) and s.nav.moving(th.v_eps, th.w_eps):
        if not s.auto.fresh(th.cmd_fresh_sec):
            if s.dry_run.fresh(th.cmd_fresh_sec):
                findings.append('smoother 출력이 /cmd_vel_nav_dry_run 으로 감: motion_enabled:=false')
            else:
                findings.append('/cmd_vel_nav 는 있는데 /cmd_vel_auto 가 없음: velocity_smoother'
                                ' 비활성 또는 remap 불일치')

    # 3) Safety 단계
    if s.auto.fresh(th.cmd_fresh_sec) and s.auto.moving(th.v_eps, th.w_eps):
        if not s.safe.moving(th.v_eps, th.w_eps):
            reason = safety_reason(s.safety_state)
            category = safety_category(s.safety_state)
            if reason:
                findings.append(f'Safety 가 차단: {reason} (분류={category})')
            elif s.auto.v < -th.v_eps and abs(s.auto.w) <= th.w_eps:
                findings.append('후진 명령이 max_reverse_velocity=0.0 으로 0 이 됨 (설계상 차단)')
    if s.estop_age is None:
        findings.append('/estop_state 를 한 번도 못 받음')
    elif s.estop_age > th.state_fresh_sec:
        findings.append('/estop_state 끊김 (발행자 중단 또는 QoS/도메인 불일치)')
    elif s.estop:
        findings.append('E-stop 활성(/estop_state=true)')

    if s.obstacle_age is None:
        findings.append('/safety/obstacle_detected 를 한 번도 못 받음 (lidar_obstacle_node)')
    elif s.obstacle_age > th.state_fresh_sec:
        findings.append('/safety/obstacle_detected 끊김 (lidar_obstacle_node 중단?)')

    # 4) 구동 단계: 안전 출력이 움직이라는데 바퀴가 안 움직임
    if s.safe.fresh(th.cmd_fresh_sec) and s.safe.moving(th.v_eps, th.w_eps):
        if not s.wheel.fresh(th.sensor_fresh_sec):
            findings.append('/wheel/odom 없음: Gazebo bridge 또는 실물 bridge 확인')
        elif not s.wheel.moving(th.v_eps * 0.5, th.w_eps * 0.5):
            findings.append('/cmd_vel_safe 는 움직이라는데 /wheel/odom 이 0: 구동 플러그인/bridge/물리 확인')

    # 5) EKF 단계
    if s.wheel.fresh(th.sensor_fresh_sec) and s.wheel.moving(th.v_eps, th.w_eps):
        if not s.odom.fresh(th.sensor_fresh_sec):
            findings.append('/odom(EKF) 없음 또는 끊김')
        elif not s.odom.moving(th.v_eps * 0.5, th.w_eps * 0.5):
            findings.append('/wheel/odom 은 움직이는데 /odom 이 0: EKF 입력 거부(frame/공분산/시간) 의심')

    # 6) TF
    if s.tf_odom_base_ok is False:
        findings.append('TF odom->base_link 조회 실패 (EKF publish_tf 확인)')
    if s.tf_map_odom_ok is False:
        findings.append('TF map->odom 조회 실패 (AMCL 초기 위치/스캔 확인)')

    return findings


# ----------------------------------------------------------------------
# 회전 비교: 실제 회전 명령 구간에서만 yaw 를 적분한다.
# ----------------------------------------------------------------------
@dataclass
class RotationSegment:
    """한 번의 연속 회전 명령 구간 동안 출처별 yaw 적분값(rad)."""

    start: float
    duration: float = 0.0
    yaw: Dict[str, float] = field(default_factory=dict)

    def ratio(self, numerator: str, denominator: str) -> Optional[float]:
        a = self.yaw.get(numerator)
        b = self.yaw.get(denominator)
        if a is None or b is None or abs(b) < 1e-3:
            return None
        return a / b


class RotationComparator:
    """
    |cmd_w| >= w_on 인 동안 각 출처의 각속도를 적분한다.

    sources: 'cmd'(= /cmd_vel_safe), 'wheel', 'imu', 'odom', 'truth'(있을 때만).
    정지 중 0 끼리의 비교는 구간이 열리지 않으므로 결과에 들어가지 않는다.
    """

    def __init__(self, w_on: float = 0.1, min_duration: float = 1.0):
        self.w_on = w_on
        self.min_duration = min_duration
        self.segments: List[RotationSegment] = []
        self._open: Optional[RotationSegment] = None
        self._last_t: Optional[float] = None

    def update(self, t: float, rates: Dict[str, Optional[float]]) -> Optional[RotationSegment]:
        """t: 단조 증가 시각(s). rates 에서 None 인 출처는 이번 구간에서 적분하지 않는다."""
        cmd = rates.get('cmd')
        closed = None
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t

        active = cmd is not None and abs(cmd) >= self.w_on
        if active and self._open is None:
            self._open = RotationSegment(start=t)
        elif not active and self._open is not None:
            if self._open.duration >= self.min_duration:
                self.segments.append(self._open)
                closed = self._open
            self._open = None

        if self._open is not None and dt > 0.0:
            self._open.duration += dt
            for name, rate in rates.items():
                if rate is None or not math.isfinite(rate):
                    continue
                self._open.yaw[name] = self._open.yaw.get(name, 0.0) + rate * dt
        return closed

    def summary(self) -> Dict[str, Optional[float]]:
        """모든 완료 구간을 합친 비율. 기준(분모)은 imu, truth 가 있으면 truth 도 함께."""
        total: Dict[str, float] = {}
        duration = 0.0
        for seg in self.segments:
            duration += seg.duration
            for name, value in seg.yaw.items():
                total[name] = total.get(name, 0.0) + value
        result: Dict[str, Optional[float]] = {'segments': float(len(self.segments)),
                                              'duration_sec': duration}
        for name, value in total.items():
            result[f'yaw_{name}_rad'] = value
        for ref in ('imu', 'truth'):
            if abs(total.get(ref, 0.0)) < 1e-3:
                continue
            for name in ('cmd', 'wheel', 'odom', 'imu'):
                if name != ref and name in total:
                    result[f'{name}/{ref}'] = total[name] / total[ref]
        return result


def interpret_rotation(summary: Dict[str, Optional[float]]) -> List[str]:
    """회전 비교 요약을 해석한다. 비율 근거가 없으면 판정하지 않는다."""
    notes: List[str] = []
    if not summary.get('segments'):
        return ['회전 명령 구간이 없어 EKF/휠/IMU 회전 비교를 판정하지 않음']
    ref = 'truth' if summary.get('yaw_truth_rad') else 'imu'
    wheel = summary.get(f'wheel/{ref}')
    odom = summary.get(f'odom/{ref}')
    cmd = summary.get(f'cmd/{ref}')
    if cmd is not None and abs(cmd - 1.0) > 0.2:
        notes.append(f'명령 대비 실제 회전 비 cmd/{ref}={cmd:.2f}: 차체가 명령만큼 돌지 않음'
                     ' (스키드 미끄럼/가속 제한/마찰)')
    if wheel is not None and abs(wheel - 1.0) > 0.15:
        notes.append(f'바퀴 오도메트리 회전 비 wheel/{ref}={wheel:.2f}: 휠 기반 yaw rate 가 '
                     '실제와 다름 (스키드 슬립이면 >1)')
    if odom is not None:
        if abs(odom - 1.0) <= 0.1:
            notes.append(f'EKF /odom 회전은 기준과 일치 (odom/{ref}={odom:.2f})')
        elif wheel is not None and abs(odom - wheel) < 0.1:
            notes.append(f'EKF /odom 이 바퀴 yaw rate 를 따라감 (odom/{ref}={odom:.2f}):'
                         ' wheel vyaw 융합 설정 영향')
        else:
            notes.append(f'EKF /odom 회전 비 odom/{ref}={odom:.2f}')
    return notes
