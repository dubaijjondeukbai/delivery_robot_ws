"""
가짜 STM32 (가상 시리얼 pty): 펌웨어 없이 base_bridge_node 와 전체 명령 흐름을 시험한다.

ROS 가 필요 없다.

    ros2 run robot_base mock_stm32                 # /tmp/ttySTM32 에 가상 포트를 만든다
    ros2 run robot_base mock_stm32 --max-steer-deg 60 --no-steer-feedback

흉내 내는 것 (펌웨어가 지켜야 할 동작과 같다, docs/stm32_drive_protocol.md):
* DRV/VEL 를 받으면 조향 서보(속도 제한 --steer-rate-dps, 범위 --max-steer-deg)와
  바퀴 속도(1차 지연 --wheel-tau)를 목표로 움직인다.
* 0.3 s 동안 DRV/VEL 이 없으면 바퀴를 세운다(워치독). 조향각은 그대로 둔다.
* FB 50 Hz, EST 10 Hz. --estop-file 파일 내용이 '1' 이면 EST,1 (E-stop 눌림).
* 팀원 명령: STOP -> ACK,STOP, ESTOP -> STATE,EMERGENCY_STOP (RESET 까지 바퀴 정지)
  바퀴가 서 있다가 움직이면 DRIVE_MOVING, 멈추면 DRIVE_STOPPED (바뀔 때만).
* 2 초마다 가짜 실제 위치(바퀴 상태를 적분한 값)를 표준출력에 찍는다.
"""

from __future__ import annotations

import argparse
import math
import os
import random
import select
import sys
import time
import tty
from typing import List, Optional

from robot_base import protocol, swerve

MOVING_RPM = 0.5


class MockStm32:

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.geometry = swerve.SwerveGeometry(
            args.wheel_x, args.wheel_y, args.wheel_radius, steer_mode=args.steer_mode)
        self.limits = swerve.SwerveLimits(
            max_steer=math.radians(args.max_steer_deg),
            max_wheel_speed=self.geometry.rpm_to_speed(args.max_wheel_rpm),
        )
        self.angles = [0.0] * 4
        self.rpms = [0.0] * 4
        self.target_angles = [0.0] * 4
        self.target_rpms = [0.0] * 4
        self.last_cmd = -1e9
        self.latched = False
        self.moving = False
        self.rx = bytearray()
        self.start = time.monotonic()
        self.pose = [0.0, 0.0, 0.0]
        self.drv_count = 0
        self.rng = random.Random(1)

        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        self.slave_name = os.ttyname(self.slave)
        if args.link:
            if os.path.islink(args.link) or os.path.exists(args.link):
                os.remove(args.link)
            os.symlink(self.slave_name, args.link)
        print(f'[mock_stm32] port {self.slave_name} -> {args.link or "(no link)"}', flush=True)

    # ------------------------------------------------------------------
    def send(self, body: str) -> None:
        try:
            os.write(self.master, protocol.frame(body, self.args.checksum))
        except OSError:
            pass

    def handle(self, line: str) -> None:
        parsed_body = line.strip()
        if '*' in parsed_body and len(parsed_body) >= 3 and parsed_body[-3] == '*':
            parsed_body = parsed_body[:-3]
        fields = parsed_body.split(',')
        head = fields[0].upper()
        now = time.monotonic()
        if head == 'DRV' and len(fields) == 10:
            try:
                values = [int(v) for v in fields[2:]]
            except ValueError:
                return
            self.target_angles = [
                max(-self.limits.max_steer, min(self.limits.max_steer, math.radians(v / 10.0)))
                if steer else 0.0  # 뒷바퀴 서보 없음: 0° 고정
                for v, steer in zip(values[:4], self.geometry.steerable())]
            self.target_rpms = [
                max(-self.args.max_wheel_rpm, min(self.args.max_wheel_rpm, v / 10.0))
                for v in values[4:]]
            self.last_cmd = now
            self.drv_count += 1
        elif head == 'VEL' and len(fields) == 5:
            try:
                vx, vy, wz = (int(v) / 1000.0 for v in fields[2:])
            except ValueError:
                return
            result = swerve.inverse(self.geometry, self.limits, vx, vy, wz, self.angles)
            self.target_angles = [w.angle for w in result.wheels]
            self.target_rpms = [self.geometry.speed_to_rpm(w.speed) for w in result.wheels]
            self.last_cmd = now
            self.drv_count += 1
        elif head == 'STOP':
            self.target_rpms = [0.0] * 4
            self.send('ACK,STOP')
        elif head == 'ESTOP':
            self.latched = True
            self.send('STATE,EMERGENCY_STOP')
        elif head == 'RESET':
            self.latched = False
            self.send('ACK,RESET')
        elif head:
            self.send('ERROR,INVALID_COMMAND')

    def estop_pressed(self) -> bool:
        path = self.args.estop_file
        if not path:
            return False
        try:
            with open(path, encoding='utf-8') as handle:
                return handle.read().strip() == '1'
        except OSError:
            return False

    def step(self, dt: float) -> None:
        now = time.monotonic()
        stop = self.latched or self.estop_pressed() or now - self.last_cmd > self.args.watchdog
        targets = [0.0] * 4 if stop else self.target_rpms
        steer_step = math.radians(self.args.steer_rate_dps) * dt
        alpha = 1.0 - math.exp(-dt / self.args.wheel_tau)
        for i in range(4):
            error = swerve.wrap_angle(self.target_angles[i] - self.angles[i])
            self.angles[i] += max(-steer_step, min(steer_step, error))
            self.rpms[i] += (targets[i] - self.rpms[i]) * alpha

        speeds = [self.geometry.rpm_to_speed(r) for r in self.rpms]
        vx, vy, wz, _ = swerve.forward(self.geometry, self.angles, speeds)
        yaw = self.pose[2] + 0.5 * wz * dt
        self.pose[0] += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        self.pose[1] += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        self.pose[2] = swerve.wrap_angle(self.pose[2] + wz * dt)

        moving = max(abs(r) for r in self.rpms) > MOVING_RPM
        if moving != self.moving:
            self.moving = moving
            self.send('DRIVE_MOVING' if moving else 'DRIVE_STOPPED')

    def feedback(self) -> None:
        t_ms = int((time.monotonic() - self.start) * 1000.0) % (2 ** 32)
        noise = self.args.noise
        angles = self.angles
        if not self.args.steer_feedback:
            angles = self.target_angles  # 서보 측정값이 없는 펌웨어: 명령값을 돌려준다
        angles = [a + self.rng.gauss(0.0, math.radians(0.2)) * noise for a in angles]
        rpms = [r + self.rng.gauss(0.0, 0.2) * noise for r in self.rpms]
        values: List[int] = [t_ms]
        values += [int(round(math.degrees(a) * 10.0)) for a in angles]
        values += [int(round(r * 10.0)) for r in rpms]
        self.send('FB,' + ','.join(str(v) for v in values))

    def run(self) -> None:
        period = 0.005
        next_fb = next_est = next_print = time.monotonic()
        last = time.monotonic()
        while True:
            ready, _, _ = select.select([self.master], [], [], period)
            if ready:
                try:
                    data = os.read(self.master, 4096)
                except OSError:
                    data = b''
                self.rx.extend(data)
                while b'\n' in self.rx:
                    line, _, rest = bytes(self.rx).partition(b'\n')
                    self.rx = bytearray(rest)
                    self.handle(line.decode('ascii', errors='replace'))
            now = time.monotonic()
            self.step(now - last)
            last = now
            if now >= next_fb:
                next_fb += 1.0 / self.args.fb_rate
                self.feedback()
            if now >= next_est:
                next_est += 0.1
                self.send(f'EST,{int(self.estop_pressed())}')
            if now >= next_print:
                next_print += 2.0
                print(
                    f'[mock_stm32] truth x={self.pose[0]:.3f} y={self.pose[1]:.3f} '
                    f'yaw={math.degrees(self.pose[2]):.1f}deg drv={self.drv_count} '
                    f'angles={[round(math.degrees(a)) for a in self.angles]} '
                    f'rpm={[round(r, 1) for r in self.rpms]} latched={int(self.latched)}',
                    flush=True)
                if self.args.truth_file:
                    with open(self.args.truth_file, 'w', encoding='utf-8') as handle:
                        handle.write(f'{self.pose[0]} {self.pose[1]} {self.pose[2]}\n')


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Fake STM32 for robot_base (pty)')
    parser.add_argument('--link', default='/tmp/ttySTM32')
    parser.add_argument('--wheel-x', type=float, default=0.230)
    parser.add_argument('--wheel-y', type=float, default=0.2706)
    parser.add_argument('--wheel-radius', type=float, default=0.0857)
    parser.add_argument('--steer-mode', choices=swerve.STEER_MODES, default='front')
    parser.add_argument('--max-steer-deg', type=float, default=30.0)
    parser.add_argument('--max-wheel-rpm', type=float, default=150.0)
    parser.add_argument('--steer-rate-dps', type=float, default=180.0)
    parser.add_argument('--wheel-tau', type=float, default=0.15)
    parser.add_argument('--watchdog', type=float, default=0.3)
    parser.add_argument('--fb-rate', type=float, default=50.0)
    parser.add_argument('--noise', type=float, default=0.0)
    parser.add_argument('--no-steer-feedback', dest='steer_feedback', action='store_false')
    parser.add_argument('--checksum', action='store_true')
    parser.add_argument('--estop-file', default='')
    parser.add_argument('--truth-file', default='')
    # ros2 run/launch 가 붙이는 --ros-args 이하를 무시한다
    argv = list(sys.argv[1:] if argv is None else argv)
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> None:
    args = parse_args(argv)
    mock = MockStm32(args)
    try:
        mock.run()
    except KeyboardInterrupt:
        pass
    finally:
        if args.link and os.path.islink(args.link):
            os.remove(args.link)


if __name__ == '__main__':
    main()
