"""
STM32 시리얼 줄 단위 프로토콜. ROS 의존성 없음, 단위 시험 대상.

팀원 코드(delivery_robot_bridge)와 같은 형식이다: ASCII 한 줄, 줄바꿈(LF)으로 끝, 쉼표 구분.
모든 숫자는 정수(펌웨어에서 sscanf/atoi 로 읽기 쉽도록). 바퀴 순서 FL, FR, RL, RR.

Pi -> STM32
    DRV,<seq>,<a_fl>,<a_fr>,<a_rl>,<a_rr>,<r_fl>,<r_fr>,<r_rl>,<r_rr>
        a: 조향각 [0.1 deg] (+ = 왼쪽), r: 바퀴 속도 [0.1 rpm] (+ = 바퀴가 향한 쪽으로 굴러감)
        command_mode=wheels 일 때. 20 Hz 로 계속 보낸다(정지도 0 으로 보냄).
    VEL,<seq>,<vx [mm/s]>,<vy [mm/s]>,<wz [mrad/s]>
        command_mode=body 일 때(펌웨어가 바퀴 계산을 하는 경우).
    그 밖의 줄(STOP, ESTOP 등)은 /control_command 를 그대로 보낸다.

STM32 -> Pi
    FB,<t_ms>,<a_fl>,<a_fr>,<a_rl>,<a_rr>,<r_fl>,<r_fr>,<r_rl>,<r_rr>
        측정 조향각 [0.1 deg]·바퀴 속도 [0.1 rpm], t_ms = STM32 밀리초 카운터(넘침 허용). 50 Hz 권장.
    EST,<0|1>
        E-stop 상태(1 = 눌림). 10 Hz 이상으로 계속 보낸다.
    IMU,<t_ms>,<gx>,<gy>,<gz>,<ax>,<ay>,<az>
        선택. 각속도 [mrad/s], 가속도 [mm/s^2], 센서 좌표 = imu_link.
    그 밖의 줄(ACK,STOP / DRIVE_STOPPED 등)은 /stm32_status 로 그대로 넘긴다.

선택: 줄 끝에 '*HH' 를 붙일 수 있다('*' 앞 모든 문자의 XOR, 16진 대문자 2자리, NMEA 와 같음).
붙어 있으면 검사하고, 틀리면 버린다. 보낼 때는 use_checksum 설정을 따른다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple, Union

MAX_LINE = 200
WHEELS = 4


@dataclass
class Feedback:
    t_ms: int
    angles: List[float]  # [rad]
    rpms: List[float]    # [rpm]


@dataclass
class EStop:
    active: bool


@dataclass
class ImuSample:
    t_ms: int
    gyro: Tuple[float, float, float]   # [rad/s]
    accel: Tuple[float, float, float]  # [m/s^2]


@dataclass
class Text:
    text: str


@dataclass
class Bad:
    line: str
    reason: str


Parsed = Union[Feedback, EStop, ImuSample, Text, Bad]


def checksum(body: str) -> str:
    value = 0
    for char in body.encode('ascii', errors='replace'):
        value ^= char
    return f'{value:02X}'


def frame(body: str, use_checksum: bool = False) -> bytes:
    """보낼 줄을 만든다(끝에 줄바꿈 LF)."""
    if use_checksum:
        body = f'{body}*{checksum(body)}'
    return (body + '\n').encode('ascii', errors='replace')


def _deci(value: float) -> int:
    return int(round(value * 10.0))


def format_drv(seq: int, angles: Sequence[float], rpms: Sequence[float]) -> str:
    """DRV 줄 본문을 만든다 (angles [rad], rpms [rpm])."""
    fields = [str(seq % 65536)]
    fields += [str(_deci(math.degrees(a))) for a in angles]
    fields += [str(_deci(r)) for r in rpms]
    return 'DRV,' + ','.join(fields)


def format_vel(seq: int, vx: float, vy: float, wz: float) -> str:
    """VEL 줄 본문을 만든다 (vx, vy [m/s], wz [rad/s])."""
    return 'VEL,{},{},{},{}'.format(
        seq % 65536,
        int(round(vx * 1000.0)),
        int(round(vy * 1000.0)),
        int(round(wz * 1000.0)),
    )


def _strip_checksum(line: str) -> Tuple[Optional[str], str]:
    """(본문, 오류). '*HH' 가 있으면 검사한다."""
    if len(line) >= 3 and line[-3] == '*':
        body, given = line[:-3], line[-2:].upper()
        try:
            int(given, 16)
        except ValueError:
            return None, 'bad_checksum_format'
        if checksum(body) != given:
            return None, 'checksum_mismatch'
        return body, ''
    return line, ''


def _ints(fields: Sequence[str]) -> Optional[List[int]]:
    try:
        return [int(f) for f in fields]
    except ValueError:
        return None


def parse_line(raw: Union[str, bytes]) -> Parsed:
    """STM32 가 보낸 한 줄을 해석한다. 모르는 머리말은 Text 로 넘긴다."""
    if isinstance(raw, bytes):
        raw = raw.decode('utf-8', errors='replace')
    line = raw.strip()
    if not line:
        return Bad(line, 'empty')
    if len(line) > MAX_LINE:
        return Bad(line[:MAX_LINE], 'too_long')

    body, error = _strip_checksum(line)
    if body is None:
        return Bad(line, error)

    fields = body.split(',')
    head = fields[0].strip().upper()

    if head == 'FB':
        values = _ints(fields[1:])
        if values is None or len(values) != 1 + 2 * WHEELS:
            return Bad(line, 'fb_fields')
        return Feedback(
            t_ms=values[0],
            angles=[math.radians(v / 10.0) for v in values[1:1 + WHEELS]],
            rpms=[v / 10.0 for v in values[1 + WHEELS:]],
        )

    if head == 'EST':
        values = _ints(fields[1:])
        if values is None or len(values) != 1 or values[0] not in (0, 1):
            return Bad(line, 'est_fields')
        return EStop(active=values[0] == 1)

    if head == 'IMU':
        values = _ints(fields[1:])
        if values is None or len(values) != 7:
            return Bad(line, 'imu_fields')
        return ImuSample(
            t_ms=values[0],
            gyro=tuple(v / 1000.0 for v in values[1:4]),
            accel=tuple(v / 1000.0 for v in values[4:7]),
        )

    return Text(body)


def tick_delta_ms(previous: int, current: int, modulus: int = 2 ** 32) -> int:
    """STM32 밀리초 카운터 차이(넘침 처리)."""
    return (current - previous) % modulus
