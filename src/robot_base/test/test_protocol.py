import math

import pytest
from robot_base import protocol


def test_format_drv():
    body = protocol.format_drv(
        7, [0.0, math.pi / 4, -math.pi / 4, math.pi / 2], [10.0, -5.5, 0.0, 150.0])
    assert body == 'DRV,7,0,450,-450,900,100,-55,0,1500'


def test_format_vel_wraps_sequence():
    assert protocol.format_vel(65537, 0.2, 0.0, -0.5) == 'VEL,1,200,0,-500'


def test_frame_and_checksum_round_trip():
    raw = protocol.frame('FB,10,0,0,0,0,0,0,0,0', use_checksum=True)
    assert raw.endswith(b'\n')
    parsed = protocol.parse_line(raw)
    assert isinstance(parsed, protocol.Feedback)
    corrupted = raw.replace(b'FB,10', b'FB,11')
    bad = protocol.parse_line(corrupted)
    assert isinstance(bad, protocol.Bad) and bad.reason == 'checksum_mismatch'
    assert protocol.frame('STOP') == b'STOP\n'


def test_parse_feedback():
    parsed = protocol.parse_line('FB,123456,0,450,-450,900,100,-55,0,1500')
    assert isinstance(parsed, protocol.Feedback)
    assert parsed.t_ms == 123456
    assert parsed.angles == pytest.approx(
        [0.0, math.pi / 4, -math.pi / 4, math.pi / 2])
    assert parsed.rpms == pytest.approx([10.0, -5.5, 0.0, 150.0])


@pytest.mark.parametrize('line, reason', [
    ('FB,1,2,3', 'fb_fields'),
    ('FB,1,a,0,0,0,0,0,0,0', 'fb_fields'),
    ('EST,2', 'est_fields'),
    ('IMU,1,2,3', 'imu_fields'),
    ('', 'empty'),
    ('X' * 300, 'too_long'),
    ('EST,1*G1', 'bad_checksum_format'),
])
def test_parse_rejects(line, reason):
    parsed = protocol.parse_line(line)
    assert isinstance(parsed, protocol.Bad)
    assert parsed.reason == reason


def test_parse_estop_and_imu():
    assert protocol.parse_line('EST,1') == protocol.EStop(active=True)
    assert protocol.parse_line('est,0\r\n') == protocol.EStop(active=False)
    imu = protocol.parse_line('IMU,5,0,0,500,0,0,9810')
    assert isinstance(imu, protocol.ImuSample)
    assert imu.gyro == pytest.approx((0.0, 0.0, 0.5))
    assert imu.accel == pytest.approx((0.0, 0.0, 9.81))


def test_unknown_lines_pass_through_as_text():
    for line in ('ACK,STOP', 'DRIVE_STOPPED', 'STATE,EMERGENCY_STOP'):
        assert protocol.parse_line(line) == protocol.Text(line)


def test_tick_delta_wraps():
    assert protocol.tick_delta_ms(2 ** 32 - 10, 5) == 15
    assert protocol.tick_delta_ms(100, 120) == 20
