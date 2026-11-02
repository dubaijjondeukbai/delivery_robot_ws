import math
import random

import pytest
from robot_base import swerve

GEOMETRY = swerve.SwerveGeometry(wheel_x=0.230, wheel_y=0.2706, wheel_radius=0.0857)
FULL = swerve.SwerveLimits(max_steer=math.radians(90.0), max_wheel_speed=10.0)
ZERO = [0.0, 0.0, 0.0, 0.0]


def angles(result):
    return [w.angle for w in result.wheels]


def speeds(result):
    return [w.speed for w in result.wheels]


def test_straight_forward():
    result = swerve.inverse(GEOMETRY, FULL, 0.3, 0.0, 0.0, ZERO)
    assert result.adjusted == ''
    assert angles(result) == pytest.approx(ZERO)
    assert speeds(result) == pytest.approx([0.3] * 4)


def test_in_place_rotation_pattern():
    result = swerve.inverse(GEOMETRY, FULL, 0.0, 0.0, 0.5, ZERO)
    need = GEOMETRY.in_place_steer_angle()
    assert math.degrees(need) == pytest.approx(40.36, abs=0.01)
    # FL, RR 는 -need, FR, RL 은 +need. 왼쪽 바퀴(FL, RL)는 뒤로 굴러간다.
    assert angles(result) == pytest.approx([-need, need, need, -need])
    assert [s > 0 for s in speeds(result)] == [False, True, False, True]
    vx, vy, wz, residual = swerve.forward(GEOMETRY, angles(result), speeds(result))
    assert (vx, vy, wz) == pytest.approx((0.0, 0.0, 0.5), abs=1e-12)
    assert residual == pytest.approx(0.0, abs=1e-12)


def test_round_trip_random_twists():
    rng = random.Random(7)
    for _ in range(500):
        twist = (rng.uniform(-0.4, 0.4), rng.uniform(-0.2, 0.2), rng.uniform(-1.0, 1.0))
        current = [rng.uniform(-1.5, 1.5) for _ in range(4)]
        result = swerve.inverse(GEOMETRY, FULL, *twist, current)
        assert result.adjusted == ''
        assert all(abs(a) <= FULL.max_steer + 1e-9 for a in angles(result))
        vx, vy, wz, residual = swerve.forward(GEOMETRY, angles(result), speeds(result))
        assert (vx, vy, wz) == pytest.approx(twist, abs=1e-9)
        assert residual == pytest.approx(0.0, abs=1e-9)


def test_flip_prefers_angle_closest_to_current():
    # 정확히 옆으로(vy) 가는 명령은 +90°/전진 과 -90°/후진 이 모두 가능하다.
    left = swerve.inverse(GEOMETRY, FULL, 0.0, 0.2, 0.0, [1.4] * 4)
    right = swerve.inverse(GEOMETRY, FULL, 0.0, 0.2, 0.0, [-1.4] * 4)
    assert angles(left) == pytest.approx([math.pi / 2] * 4)
    assert speeds(left) == pytest.approx([0.2] * 4)
    assert angles(right) == pytest.approx([-math.pi / 2] * 4)
    assert speeds(right) == pytest.approx([-0.2] * 4)


def test_stopped_keeps_current_angle():
    current = [0.3, -0.2, 0.1, 0.0]
    result = swerve.inverse(GEOMETRY, FULL, 0.0, 0.0, 0.0, current)
    assert angles(result) == pytest.approx(current)
    assert speeds(result) == pytest.approx(ZERO)


def test_limited_steering_adjusts_command_into_range():
    limits = swerve.SwerveLimits(max_steer=math.radians(45.0), max_wheel_speed=10.0)
    band = swerve.infeasible_turn_band(GEOMETRY, limits.max_steer)
    assert band == pytest.approx((0.2706 - 0.230, 0.2706 + 0.230))
    # 회전반경 0.28 m (안쪽 바퀴 줄 근처) -> 범위 밖
    result = swerve.inverse(GEOMETRY, limits, 0.14, 0.0, 0.5, ZERO)
    assert result.adjusted in ('reduce_wz', 'reduce_v')
    assert all(abs(a) <= limits.max_steer + 1e-9 for a in angles(result))
    vx, vy, wz, _ = swerve.forward(GEOMETRY, angles(result), speeds(result))
    assert (vx, vy, wz) == pytest.approx(result.used, abs=1e-6)


def test_rotation_impossible_when_steering_range_too_small():
    limits = swerve.SwerveLimits(max_steer=math.radians(30.0), max_wheel_speed=10.0)
    result = swerve.inverse(GEOMETRY, limits, 0.0, 0.0, 0.5, ZERO)
    assert result.adjusted == 'infeasible'
    assert speeds(result) == pytest.approx(ZERO)
    assert result.used == (0.0, 0.0, 0.0)


def test_wheel_speed_limit_scales_everything():
    limits = swerve.SwerveLimits(max_steer=math.radians(90.0), max_wheel_speed=0.25)
    result = swerve.inverse(GEOMETRY, limits, 0.3, 0.0, 0.5, ZERO)
    assert max(abs(s) for s in speeds(result)) == pytest.approx(0.25)
    assert result.speed_scale < 1.0
    vx, _, wz, _ = swerve.forward(GEOMETRY, angles(result), speeds(result))
    assert wz / vx == pytest.approx(0.5 / 0.3)  # 회전 중심 유지


def test_steering_gate():
    gate = math.radians(20.0)
    assert swerve.steering_gate(ZERO, [0.5] * 4, gate) == 0.0
    assert swerve.steering_gate(ZERO, [0.1, 0.0, -0.05, 0.0], gate) == pytest.approx(math.cos(0.1))
    assert swerve.steering_gate([math.pi - 0.01], [-math.pi + 0.01], gate) == pytest.approx(
        math.cos(0.02))


def test_forward_reports_inconsistent_wheels():
    _, _, _, residual = swerve.forward(GEOMETRY, ZERO, [0.3, 0.3, 0.3, -0.3])
    assert residual > 0.1


def test_wrap_angle():
    assert swerve.wrap_angle(math.pi) == pytest.approx(math.pi)
    assert swerve.wrap_angle(-math.pi) == pytest.approx(math.pi)
    assert swerve.wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert swerve.infeasible_turn_band(GEOMETRY, math.radians(90.0)) is None


def test_rpm_conversion():
    assert GEOMETRY.speed_to_rpm(GEOMETRY.rpm_to_speed(123.4)) == pytest.approx(123.4)
    assert GEOMETRY.rpm_to_speed(60.0) == pytest.approx(2.0 * math.pi * 0.0857)


# ----------------------------------------------------------------------
# 앞 2바퀴 조향 (실물): 뒷바퀴 0° 고정, 명령은 뒷차축 중심 속도로 해석
# ----------------------------------------------------------------------
FRONT = swerve.SwerveGeometry(0.230, 0.2706, 0.0857, steer_mode='front')


def front_limits(degrees):
    return swerve.SwerveLimits(max_steer=math.radians(degrees), max_wheel_speed=10.0)


def test_front_ackermann_turn():
    result = swerve.inverse(FRONT, front_limits(30.0), 0.3, 0.0, 0.2, ZERO)
    assert result.adjusted == ''
    fl, fr, rl, rr = angles(result)
    assert math.degrees(fl) == pytest.approx(20.52, abs=0.01)  # 안쪽 앞바퀴가 더 꺾인다
    assert math.degrees(fr) == pytest.approx(14.56, abs=0.01)
    assert (rl, rr) == (0.0, 0.0)
    assert speeds(result)[2:] == pytest.approx([0.3 - 0.2 * 0.2706, 0.3 + 0.2 * 0.2706])
    vx, vy, wz, residual = swerve.forward(FRONT, angles(result), speeds(result))
    assert (vx, vy, wz) == pytest.approx((0.3, 0.2 * 0.230, 0.2))  # 차체 중심은 옆으로도 움직인다
    assert residual == pytest.approx(0.0, abs=1e-12)


def test_front_pivot_needs_59_5_degrees():
    assert math.degrees(FRONT.in_place_steer_angle()) == pytest.approx(59.53, abs=0.01)
    pivot = swerve.inverse(FRONT, front_limits(60.0), 0.0, 0.0, 0.5, ZERO)
    assert pivot.adjusted == ''
    need = FRONT.in_place_steer_angle()
    assert angles(pivot) == pytest.approx([-need, need, 0.0, 0.0])
    assert [s > 0 for s in speeds(pivot)] == [False, True, False, True]
    blocked = swerve.inverse(FRONT, front_limits(30.0), 0.0, 0.0, 0.5, ZERO)
    assert blocked.adjusted == 'infeasible'
    assert speeds(blocked) == pytest.approx(ZERO)


def test_front_30_degrees_minimum_radius_and_adjustment():
    limits = front_limits(30.0)
    band = swerve.infeasible_turn_band(FRONT, limits.max_steer)
    rear_radius = 0.2706 + 0.460 / math.tan(math.radians(30.0))
    assert band == pytest.approx((0.0, rear_radius))
    assert swerve.minimum_turning_radius(FRONT, limits.max_steer) == pytest.approx(
        math.hypot(rear_radius, 0.230))
    result = swerve.inverse(FRONT, limits, 0.3, 0.0, 0.5, ZERO)  # 뒷차축 반경 0.6 m 요구
    assert result.adjusted == 'reduce_wz'
    assert all(abs(a) <= limits.max_steer + 1e-9 for a in angles(result))
    assert result.used[2] == pytest.approx(0.3 / rear_radius, rel=1e-6)
    assert angles(result)[2:] == [0.0, 0.0]


def test_front_round_trip_and_rear_fixed():
    rng = random.Random(3)
    limits = front_limits(89.0)
    for _ in range(300):
        vx, wz = rng.uniform(0.05, 0.4), rng.uniform(-0.3, 0.3)
        result = swerve.inverse(FRONT, limits, vx, 0.0, wz, ZERO)
        assert angles(result)[2:] == [0.0, 0.0]
        got = swerve.forward(FRONT, angles(result), speeds(result))[:3]
        assert got == pytest.approx(result.used, abs=1e-9)
        assert result.used[1] == pytest.approx(result.used[2] * 0.230)


def test_minimum_radius_is_zero_when_rotation_possible():
    assert swerve.minimum_turning_radius(FRONT, math.radians(60.0)) == 0.0
    assert swerve.minimum_turning_radius(GEOMETRY, math.radians(45.0)) == 0.0
    with pytest.raises(ValueError):
        swerve.SwerveGeometry(0.2, 0.2, 0.1, steer_mode='rear')
