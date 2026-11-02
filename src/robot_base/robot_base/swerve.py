"""
조향 바퀴 4개짜리 베이스의 기구학. ROS 의존성 없음, 단위 시험 대상.

좌표계는 base_link (x 앞, y 왼쪽, 차체 중심). 바퀴 순서는 항상 FL, FR, RL, RR.
조향각 0 = 바퀴가 앞을 향함, + = 왼쪽(반시계). 바퀴 속도 + = 바퀴가 향한 방향으로 굴러감.
구동은 4바퀴 모두(인휠 모터), 조향은 steer_mode 로 고른다.

* steer_mode='front' (실물, 2026-11-02 확인): 앞 2바퀴만 조향(서보 2개), 뒷바퀴는 0° 고정.
  뒷바퀴는 옆으로 미끄러질 수 없으므로 회전 중심은 항상 뒷차축 줄 위에 있다.
  그래서 명령 (vx, wz) 를 '뒷차축 중심의 속도'로 해석하고, 차체 중심의 옆 속도는
  vy = wz * wheel_x 로 정해진다(Nav2 가 보내는 vy=0 은 이 값으로 바뀐다). 앞바퀴 각은
  왼/오른쪽이 따로 계산되므로 애커먼 조향이 저절로 된다.
  - 뒷차축 중심 기준 회전반경 R 에서 안쪽 앞바퀴 각 = atan2(L, |R - wheel_y|), L = 축거.
  - R = 0 (뒷차축 중심 제자리 회전)에는 atan2(L, wheel_y) ≈ 59.5° 가 필요하다.
* steer_mode='all': 4바퀴 모두 조향(스워브). 제자리 회전(차체 중심)에 atan2(wheel_x, wheel_y)
  ≈ 40.4° 필요.

역기구학: 몸체 속도 (vx, vy, wz) -> 바퀴마다 (조향각, 바퀴 둘레 속도)
    v_i = (vx - wz * y_i,  vy + wz * x_i)
정기구학(오도메트리): 바퀴마다 측정한 (조향각, 속도) -> 최소제곱 (vx, vy, wz)

조향 범위가 ±max_steer 로 제한되므로 각 바퀴는 (a, s) 와 (a + 180°, -s) 중 범위 안에 있고
지금 각도와 가까운 쪽을 고른다(바퀴 방향 반전). 범위 밖이 되는 회전반경 구간
(infeasible_turn_band)의 명령은 가장 가까운 실행 가능 명령으로 줄인다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

WHEEL_NAMES = ('fl', 'fr', 'rl', 'rr')

# 이보다 느린 바퀴는 '정지'로 보고 조향각을 바꾸지 않는다 [m/s]
STOPPED_SPEED = 1e-4


STEER_MODES = ('front', 'all')


@dataclass(frozen=True)
class SwerveGeometry:
    """바퀴 배치. 위치는 base_link 기준."""

    wheel_x: float       # 앞뒤 바퀴 중심 간격의 절반 [m]
    wheel_y: float       # 좌우 바퀴 중심 간격의 절반 [m]
    wheel_radius: float  # [m]
    steer_mode: str = 'all'  # 'front' = 앞 2바퀴만 조향, 'all' = 4바퀴 조향

    def __post_init__(self) -> None:
        if self.steer_mode not in STEER_MODES:
            raise ValueError(f'steer_mode must be one of {STEER_MODES}')

    @property
    def front_only(self) -> bool:
        return self.steer_mode == 'front'

    def positions(self) -> List[Tuple[float, float]]:
        x, y = self.wheel_x, self.wheel_y
        return [(x, y), (x, -y), (-x, y), (-x, -y)]

    def steerable(self) -> List[bool]:
        return [True, True, not self.front_only, not self.front_only]

    def in_place_steer_angle(self) -> float:
        """제자리 회전에 필요한 조향각 크기 [rad] (front: 뒷차축 중심, all: 차체 중심)."""
        if self.front_only:
            return math.atan2(2.0 * self.wheel_x, self.wheel_y)
        return math.atan2(self.wheel_x, self.wheel_y)

    def feasible_twist(self, vx: float, vy: float, wz: float) -> Tuple[float, float, float]:
        """앞바퀴 조향이면 뒷바퀴가 옆으로 미끄러지지 않도록 vy 를 정한다."""
        if self.front_only:
            return vx, wz * self.wheel_x, wz
        return vx, vy, wz

    def rpm_to_speed(self, rpm: float) -> float:
        return rpm * 2.0 * math.pi * self.wheel_radius / 60.0

    def speed_to_rpm(self, speed: float) -> float:
        return speed * 60.0 / (2.0 * math.pi * self.wheel_radius)


@dataclass(frozen=True)
class SwerveLimits:
    """조향 범위와 바퀴 최고 속도."""

    max_steer: float        # 대칭 조향 범위 ±max_steer [rad]
    max_wheel_speed: float  # 바퀴 둘레 속도 [m/s]


@dataclass
class WheelCommand:
    angle: float  # [rad]
    speed: float  # 바퀴 둘레 속도 [m/s], + = angle 방향


@dataclass
class InverseResult:
    """역기구학 결과와 명령을 바꿨는지 여부."""

    wheels: List[WheelCommand]
    used: Tuple[float, float, float]  # 실제로 바퀴에 반영한 (vx, vy, wz)
    adjusted: str = ''                # '' | 'reduce_wz' | 'reduce_v' | 'infeasible'
    speed_scale: float = 1.0          # 바퀴 최고 속도 때문에 곱한 배율


def wrap_angle(angle: float) -> float:
    """(-pi, pi] 로 감싼다."""
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def body_to_wheel_vectors(
    geometry: SwerveGeometry, vx: float, vy: float, wz: float,
) -> List[Tuple[float, float]]:
    return [(vx - wz * y, vy + wz * x) for x, y in geometry.positions()]


def solve_wheel(
    vector: Tuple[float, float], current_angle: float, max_steer: float,
) -> Optional[WheelCommand]:
    """바퀴 하나의 속도 벡터를 조향 범위 안의 (각도, 속도)로 바꾼다. 불가능하면 None."""
    vx, vy = vector
    speed = math.hypot(vx, vy)
    if speed < STOPPED_SPEED:
        keep = max(-max_steer, min(max_steer, current_angle))
        return WheelCommand(keep, 0.0)
    angle = math.atan2(vy, vx)
    candidates = (
        WheelCommand(angle, speed),
        WheelCommand(wrap_angle(angle + math.pi), -speed),
    )
    feasible = [c for c in candidates if abs(c.angle) <= max_steer + 1e-9]
    if not feasible:
        return None
    return min(feasible, key=lambda c: abs(wrap_angle(c.angle - current_angle)))


def _solve_all(
    geometry: SwerveGeometry, limits: SwerveLimits,
    twist: Tuple[float, float, float], current: Sequence[float],
) -> Optional[List[WheelCommand]]:
    """바퀴 4개를 푼다. 입력 twist 는 feasible_twist 를 거친 값이어야 한다."""
    wheels = []
    vectors = body_to_wheel_vectors(geometry, *twist)
    for vector, angle, steerable in zip(vectors, current, geometry.steerable()):
        limit = limits.max_steer if steerable else 0.0
        command = solve_wheel(vector, angle if steerable else 0.0, limit)
        if command is None:
            return None
        if not steerable:
            command.angle = 0.0
        wheels.append(command)
    return wheels


def _largest_feasible_scale(test, iterations: int = 30) -> float:
    """test(0) 은 참, test(1) 은 거짓일 때 참인 가장 큰 배율 (이분 탐색)."""
    low, high = 0.0, 1.0
    for _ in range(iterations):
        middle = 0.5 * (low + high)
        if test(middle):
            low = middle
        else:
            high = middle
    return low


def inverse(
    geometry: SwerveGeometry,
    limits: SwerveLimits,
    vx: float,
    vy: float,
    wz: float,
    current_angles: Sequence[float],
) -> InverseResult:
    """몸체 속도를 바퀴 명령으로 바꾼다. 실행할 수 없는 명령은 가장 가까운 실행 가능 명령으로 줄인다."""
    def shaped(candidate):
        return geometry.feasible_twist(*candidate)

    twist = shaped((vx, vy, wz))
    wheels = _solve_all(geometry, limits, twist, current_angles)
    adjusted = ''

    if wheels is None:
        def feasible(candidate):
            return _solve_all(geometry, limits, shaped(candidate), current_angles) is not None

        options = []
        # A: 회전을 줄인다(회전 중심을 바깥으로) -> 전진은 유지
        if feasible((vx, vy, 0.0)):
            k = _largest_feasible_scale(lambda s: feasible((vx, vy, s * wz)))
            lever = 2.0 * geometry.wheel_x if geometry.front_only else geometry.wheel_x
            reach = math.hypot(lever, geometry.wheel_y)
            options.append(((1.0 - k) * abs(wz) * reach, 'reduce_wz', (vx, vy, k * wz)))
        # B: 이동을 줄인다(회전 중심을 안쪽으로) -> 회전은 유지
        if feasible((0.0, 0.0, wz)):
            m = _largest_feasible_scale(lambda s: feasible((s * vx, s * vy, wz)))
            options.append(((1.0 - m) * math.hypot(vx, vy), 'reduce_v', (m * vx, m * vy, wz)))

        if options:
            _, adjusted, chosen = min(options, key=lambda option: option[0])
            twist = shaped(chosen)
            wheels = _solve_all(geometry, limits, twist, current_angles)
            if wheels is not None and max(abs(w.speed) for w in wheels) < STOPPED_SPEED:
                wheels = None  # 줄이다 보니 사실상 0: 실행 불가로 처리
        if wheels is None:
            # 조향 범위가 제자리 회전 각보다 작으면 여기로 온다: 바퀴를 세운다.
            adjusted = 'infeasible'
            twist = (0.0, 0.0, 0.0)
            wheels = [
                WheelCommand(max(-limits.max_steer, min(limits.max_steer, a)) if steer else 0.0,
                             0.0)
                for a, steer in zip(current_angles, geometry.steerable())
            ]

    scale = 1.0
    fastest = max(abs(w.speed) for w in wheels)
    if fastest > limits.max_wheel_speed > 0.0:
        scale = limits.max_wheel_speed / fastest
        for wheel in wheels:
            wheel.speed *= scale
        twist = (twist[0] * scale, twist[1] * scale, twist[2] * scale)

    return InverseResult(wheels=wheels, used=twist, adjusted=adjusted, speed_scale=scale)


def steering_gate(
    commanded: Sequence[float], measured: Sequence[float], gate: float,
) -> float:
    """
    조향이 따라오기 전에 바퀴를 굴리지 않도록 곱할 배율 [0, 1].

    가장 큰 조향 오차가 gate 를 넘으면 0 (조향만 먼저 돌린다), 아니면 바퀴들 중 가장 작은 cos(오차).
    모든 바퀴에 같은 배율을 곱하므로 회전 중심은 유지된다.
    """
    errors = [abs(wrap_angle(c - m)) for c, m in zip(commanded, measured)]
    if not errors:
        return 1.0
    if max(errors) > gate:
        return 0.0
    return max(0.0, min(math.cos(error) for error in errors))


def forward(
    geometry: SwerveGeometry, angles: Sequence[float], speeds: Sequence[float],
) -> Tuple[float, float, float, float]:
    """측정한 바퀴 (조향각, 둘레 속도) -> 최소제곱 몸체 속도 (vx, vy, wz, 잔차 RMS [m/s])."""
    positions = geometry.positions()
    vectors = [(s * math.cos(a), s * math.sin(a)) for a, s in zip(angles, speeds)]
    count = len(positions)
    mean_x = sum(p[0] for p in positions) / count
    mean_y = sum(p[1] for p in positions) / count
    mean_vx = sum(v[0] for v in vectors) / count
    mean_vy = sum(v[1] for v in vectors) / count

    numerator = 0.0
    denominator = 0.0
    for (x, y), (wx, wy) in zip(positions, vectors):
        dx, dy = x - mean_x, y - mean_y
        numerator += dx * wy - dy * wx
        denominator += dx * dx + dy * dy
    wz = numerator / denominator if denominator > 0.0 else 0.0
    vx = mean_vx + wz * mean_y
    vy = mean_vy - wz * mean_x

    squared = 0.0
    for (px, py), (ex, ey) in zip(body_to_wheel_vectors(geometry, vx, vy, wz), vectors):
        squared += (px - ex) ** 2 + (py - ey) ** 2
    residual = math.sqrt(squared / count)
    return vx, vy, wz, residual


def infeasible_turn_band(
    geometry: SwerveGeometry, max_steer: float,
) -> Optional[Tuple[float, float]]:
    """
    조향 범위 때문에 실행할 수 없는 회전반경 구간 [m] (없으면 None).

    all  : 회전반경 R = |v / wz| (차체 중심), 안쪽 바퀴 각 = atan2(wheel_x, |R - wheel_y|)
    front: 회전반경 R = |v / wz| (뒷차축 중심), 안쪽 앞바퀴 각 = atan2(2*wheel_x, |R - wheel_y|)
    """
    if max_steer >= math.pi / 2.0 - 1e-9:
        return None
    lever = 2.0 * geometry.wheel_x if geometry.front_only else geometry.wheel_x
    half_width = lever / math.tan(max_steer)
    low = max(0.0, geometry.wheel_y - half_width)
    return low, geometry.wheel_y + half_width


def minimum_turning_radius(geometry: SwerveGeometry, max_steer: float) -> float:
    """제자리 회전이 안 될 때 차체 중심(base_link)이 그리는 가장 작은 원의 반지름 [m]. 되면 0."""
    if max_steer >= geometry.in_place_steer_angle() - 1e-9:
        return 0.0
    band = infeasible_turn_band(geometry, max_steer)
    edge = band[1] if band is not None else 0.0
    if geometry.front_only:
        return math.hypot(edge, geometry.wheel_x)  # 뒷차축 중심 반경 -> 차체 중심 반경
    return edge
