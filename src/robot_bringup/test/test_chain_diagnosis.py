"""chain_diagnosis 의 순수 함수 시험 (ROS 실행 불필요)."""

from robot_bringup.chain_diagnosis import (
    classify_rosout,
    diagnose,
    interpret_rotation,
    lidar_stop_reason,
    RotationComparator,
    safety_category,
    safety_reason,
    Snapshot,
    TwistSample,
)


def _fresh(v=0.0, w=0.0):
    return TwistSample(v, w, 0.05)


def _base_snapshot(**kw):
    s = Snapshot(safety_state='READY', safety_age=0.05, estop=False, estop_age=0.05,
                 obstacle=False, obstacle_age=0.05, tf_map_odom_ok=True, tf_odom_base_ok=True)
    for k, v in kw.items():
        setattr(s, k, v)
    return s


def test_safety_reason_and_category():
    assert safety_reason('READY') == ''
    assert safety_reason('STOP:estop_state_timeout') == 'estop_state_timeout'
    assert safety_category('STOP:estop_state_timeout') == 'estop'
    assert safety_category('STOP:obstacle_detected') == 'obstacle'
    assert safety_category('STOP:command_timeout') == 'command'
    assert safety_category(None) == 'unknown'


def test_rosout_distinguishes_invalid_path_from_no_progress():
    assert classify_rosout('Resulting plan has 0 poses in it.') == 'INVALID_PATH_0_POSES'
    assert classify_rosout('Failed to make progress') == 'FAILED_TO_MAKE_PROGRESS'
    assert classify_rosout('Exceeded time allowance before reaching the Spin goal - Exiting Spin'
                           ) == 'RECOVERY_TIMEOUT'
    assert classify_rosout('initialPoseReceived') == 'AMCL_REINIT'
    assert classify_rosout('nothing interesting') is None


def test_safety_block_is_reported_with_reason():
    s = _base_snapshot(auto=_fresh(0.2, 0.0), safe=_fresh(0.0, 0.0),
                       safety_state='STOP:obstacle_detected')
    findings = diagnose(s)
    assert any('obstacle_detected' in f and '분류=obstacle' in f for f in findings)


def test_estop_missing_is_reported():
    s = _base_snapshot(estop=None, estop_age=None, safety_state='STOP:waiting_for_estop_state')
    assert any('/estop_state 를 한 번도 못 받음' in f for f in diagnose(s))


def test_dry_run_detected_when_motion_disabled():
    s = _base_snapshot(goal_active=True, dry_run=_fresh(0.2, 0.1))
    assert any('motion_enabled:=false' in f for f in diagnose(s))


def test_drive_gap_detected():
    s = _base_snapshot(safe=_fresh(0.2, 0.0), wheel=_fresh(0.0, 0.0), odom=_fresh(0.0, 0.0))
    assert any('/wheel/odom 이 0' in f for f in diagnose(s))


def test_ekf_gap_detected():
    s = _base_snapshot(safe=_fresh(0.2, 0.0), wheel=_fresh(0.2, 0.0), odom=_fresh(0.0, 0.0))
    assert any('EKF' in f for f in diagnose(s))


def test_healthy_chain_has_no_findings():
    s = _base_snapshot(nav=_fresh(0.2, 0.1), auto=_fresh(0.2, 0.1), safe=_fresh(0.2, 0.1),
                       wheel=_fresh(0.2, 0.1), odom=_fresh(0.2, 0.1), goal_active=True)
    assert diagnose(s) == []


def test_rotation_ignores_standstill():
    comp = RotationComparator(w_on=0.1, min_duration=0.5)
    for i in range(50):
        comp.update(i * 0.05, {'cmd': 0.0, 'wheel': 0.0, 'imu': 0.0, 'odom': 0.0})
    assert comp.summary()['segments'] == 0.0
    assert '판정하지 않음' in interpret_rotation(comp.summary())[0]


def test_rotation_ratio_detects_wheel_overestimate():
    comp = RotationComparator(w_on=0.1, min_duration=0.5)
    t = 0.0
    for _ in range(60):     # 3 s 회전: 명령 0.5, 바퀴 0.5, 실제(IMU) 0.25, EKF 는 바퀴를 따라감
        comp.update(t, {'cmd': 0.5, 'wheel': 0.5, 'imu': 0.25, 'odom': 0.5})
        t += 0.05
    comp.update(t, {'cmd': 0.0, 'wheel': 0.0, 'imu': 0.0, 'odom': 0.0})
    summary = comp.summary()
    assert summary['segments'] == 1.0
    assert abs(summary['wheel/imu'] - 2.0) < 0.05
    notes = ' '.join(interpret_rotation(summary))
    assert 'wheel/imu=2.00' in notes and '바퀴 yaw rate 를 따라감' in notes


def test_lidar_stop_reason_separates_obstacle_from_stale_data():
    assert lidar_stop_reason('[LiDAR] STOP: obstacle_in_stop_zone') == 'obstacle_in_stop_zone'
    assert lidar_stop_reason('[LiDAR] STOP: scan_timeout') == 'scan_timeout'
    assert lidar_stop_reason('[LiDAR] CLEAR: front sector clear') is None
