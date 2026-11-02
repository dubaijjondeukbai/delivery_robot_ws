"""
조향 베이스 연결 노드 (실물 전용: 앞 2바퀴 조향 + 인휠 모터 4, 시뮬레이션에서는 쓰지 않는다).

    /cmd_vel_safe (Twist) -> 역기구학(4바퀴 조향각·속도) -> DRV 줄 -> STM32
    STM32 FB 줄 -> 정기구학 -> /wheel/odom (TF 는 발행하지 않음, odom->base_link 는 EKF 담당)
    STM32 EST 줄 -> /estop_state (Bool, 주기 발행. STM32 소식이 끊기면 true)
    STM32 IMU 줄 -> /imu/data (선택)
    /control_command (String) -> 그대로 STM32 (팀원 상태기계의 STOP, ESTOP 등)
    STM32 의 그 밖의 줄 -> /stm32_status (String)
    링크 상태 -> /bridge_status (COMM_OK / COMM_FAILED, 바뀔 때만)

시리얼 포트는 이 노드 하나만 연다. 팀원의 stm32_serial_bridge 와 함께 띄우지 않는다
(그 노드의 /control_command -> STM32, STM32 -> /stm32_status 기능을 포함한다).
줄 형식은 robot_base/protocol.py, 펌웨어 쪽 요구 사항은 docs/stm32_drive_protocol.md.
"""

from __future__ import annotations

import fcntl
import math
import termios
import time
from typing import List, Optional, Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from robot_base import protocol, swerve
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

try:
    import serial
except ImportError:  # pragma: no cover - 설치 안내만 한다
    serial = None

ZERO_TWIST = (0.0, 0.0, 0.0)
BIG_VARIANCE = 1e6


def _reliable(depth: int) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
    )


class BaseBridge(Node):

    def __init__(self) -> None:
        super().__init__('base_bridge_node')
        p = self._declare_parameters()
        self.p = p

        if serial is None:
            raise RuntimeError(
                'pyserial 이 없습니다: sudo apt install python3-serial')
        if p['command_mode'] not in ('wheels', 'body'):
            raise RuntimeError('command_mode must be wheels or body')
        if p['estop_source'] not in ('stm32', 'link'):
            raise RuntimeError('estop_source must be stm32 or link')
        if min(p['wheel_x'], p['wheel_y'], p['wheel_radius']) <= 0.0:
            raise RuntimeError('wheel_x, wheel_y, wheel_radius must be > 0')
        if not 0.0 < p['max_steer_deg'] <= 180.0:
            raise RuntimeError('max_steer_deg must be in (0, 180]')
        if p['steer_mode'] not in swerve.STEER_MODES:
            raise RuntimeError(f'steer_mode must be one of {swerve.STEER_MODES}')

        self.geometry = swerve.SwerveGeometry(
            p['wheel_x'], p['wheel_y'], p['wheel_radius'], steer_mode=p['steer_mode'])
        self.limits = swerve.SwerveLimits(
            max_steer=math.radians(p['max_steer_deg']),
            max_wheel_speed=self.geometry.rpm_to_speed(p['max_wheel_rpm']),
        )
        self.steer_gate = math.radians(p['steer_gate_deg'])

        # 명령 상태
        self.cmd = ZERO_TWIST
        self.cmd_mono = -1e9
        self.seq = 0
        self.last_used = ZERO_TWIST
        self.last_gate = 1.0
        self.last_adjusted = ''
        self.model_angles = [0.0] * 4
        self.last_control_mono = time.monotonic()

        # 링크·E-stop 상태
        self.ser = None
        self.rx = bytearray()
        self.next_open_mono = 0.0
        self.start_mono = time.monotonic()
        self.last_fb_mono: Optional[float] = None
        self.ever_linked = False
        self.reported = ''
        self.bridge_subscribers = 0
        self.stm32_estop = False
        self.last_est_mono: Optional[float] = None
        self.estop_latched = False

        # 오도메트리
        self.measured_angles: Optional[List[float]] = None
        self.last_fb_t_ms: Optional[int] = None
        self.pose = [0.0, 0.0, 0.0]
        self.last_twist = ZERO_TWIST
        self.last_residual = 0.0

        # 통계(heartbeat)
        self.fb_count = 0
        self.bad_count = 0
        self.tx_fail = 0
        self.adjusted_count = 0
        self.window_start = time.monotonic()

        self.odom_pub = self.create_publisher(Odometry, p['odom_topic'], _reliable(10))
        self.estop_pub = self.create_publisher(Bool, p['estop_topic'], _reliable(10))
        self.imu_pub = self.create_publisher(Imu, p['imu_topic'], _reliable(10))
        self.status_pub = self.create_publisher(String, p['stm32_status_topic'], 10)
        self.bridge_pub = self.create_publisher(String, p['bridge_status_topic'], 10)

        self.create_subscription(Twist, p['cmd_vel_topic'], self._on_cmd, _reliable(1))
        if p['passthrough_control']:
            self.create_subscription(
                String, p['control_command_topic'], self._on_control_command, 10)
            self.create_subscription(
                String, p['control_reset_topic'], self._on_control_reset, 10)

        self.create_timer(0.005, self._io_tick)
        self.create_timer(1.0 / p['control_rate_hz'], self._control_tick)
        self.create_timer(1.0 / p['estop_rate_hz'], self._estop_tick)
        self.create_timer(p['heartbeat_sec'], self._heartbeat)

        need = math.degrees(self.geometry.in_place_steer_angle())
        band = swerve.infeasible_turn_band(self.geometry, self.limits.max_steer)
        radius = swerve.minimum_turning_radius(self.geometry, self.limits.max_steer)
        self.get_logger().info(
            f"[Base] port={p['port']} baud={p['baudrate']} mode={p['command_mode']} "
            f"steer_mode={p['steer_mode']} "
            f"wheel_x={p['wheel_x']:.3f} wheel_y={p['wheel_y']:.4f} r={p['wheel_radius']:.4f} "
            f"max_steer={p['max_steer_deg']:.1f}deg max_rpm={p['max_wheel_rpm']:.0f} "
            f"estop_source={p['estop_source']}")
        if p['max_steer_deg'] < need:
            self.get_logger().warning(
                f'[Base] 조향 범위 {p["max_steer_deg"]:.1f}° < 제자리 회전 필요 {need:.1f}°: '
                '제자리 회전 명령은 정지로 바뀝니다(Nav2 rotate/Spin 사용 불가). '
                f'차체 중심 최소 회전반경 {radius:.2f} m')
        elif band is not None:
            self.get_logger().info(
                f'[Base] 조향 범위 때문에 회전반경 {band[0]:.2f}~{band[1]:.2f} m 명령은 '
                '가장 가까운 실행 가능 명령으로 줄입니다')
        if p['estop_source'] == 'link':
            self.get_logger().warning(
                '[Base] estop_source=link: STM32 의 EST 줄 없이 링크가 살아 있으면 '
                'E-stop 해제로 봅니다. 하드웨어 E-stop(전원 차단)이 따로 있을 때 시운전에만 쓰세요')

    # ------------------------------------------------------------------
    # 파라미터
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> dict:
        defaults = {
            'port': '/tmp/ttySTM32',
            'baudrate': 115200,
            'reconnect_sec': 1.0,
            'use_checksum': False,
            'command_mode': 'wheels',
            'control_rate_hz': 20.0,
            'cmd_timeout_sec': 0.3,
            'wheel_x': 0.230,
            'wheel_y': 0.2706,
            'wheel_radius': 0.0857,
            'steer_mode': 'front',
            'max_steer_deg': 30.0,
            'max_wheel_rpm': 150.0,
            'steer_feedback': True,
            'steer_rate_dps': 180.0,
            'steer_gate_deg': 20.0,
            'fb_timeout_sec': 0.3,
            'max_fb_gap_sec': 0.5,
            'startup_grace_sec': 5.0,
            'estop_source': 'stm32',
            'estop_rate_hz': 10.0,
            'estop_timeout_sec': 0.5,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'imu_frame': 'imu_link',
            'twist_variance': [0.0004, 0.0004, 0.0025],
            'pose_variance': [0.01, 0.01, 0.05],
            'imu_gyro_variance': 0.0004,
            'imu_accel_variance': 0.04,
            'cmd_vel_topic': '/cmd_vel_safe',
            'odom_topic': '/wheel/odom',
            'estop_topic': '/estop_state',
            'imu_topic': '/imu/data',
            'passthrough_control': True,
            'control_command_topic': '/control_command',
            'control_reset_topic': '/control_reset',
            'stm32_status_topic': '/stm32_status',
            'bridge_status_topic': '/bridge_status',
            'heartbeat_sec': 5.0,
        }
        values = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            values[name] = self.get_parameter(name).value
        for name in ('control_rate_hz', 'estop_rate_hz', 'heartbeat_sec'):
            if values[name] <= 0.0:
                raise RuntimeError(f'{name} must be > 0')
        return values

    # ------------------------------------------------------------------
    # 시리얼
    # ------------------------------------------------------------------
    def _open(self, now: float) -> None:
        if now < self.next_open_mono:
            return
        self.next_open_mono = now + self.p['reconnect_sec']
        try:
            # exclusive: 다른 프로세스(예: 팀원 stm32_serial_bridge, 두 번째 base_bridge)가
            # 같은 포트를 열어 명령이 섞이는 것을 막는다.
            self.ser = serial.Serial(
                port=self.p['port'],
                baudrate=self.p['baudrate'],
                timeout=0,
                write_timeout=0.05,
                exclusive=True,
            )
            try:
                fcntl.ioctl(self.ser.fileno(), termios.TIOCEXCL)
            except (OSError, AttributeError, ValueError):
                pass
            self.rx.clear()
            self.get_logger().info(f"[Base] serial opened (exclusive): {self.p['port']}")
        except (serial.SerialException, OSError, ValueError) as error:
            self.ser = None
            self.get_logger().warning(
                f"[Base] serial open failed ({self.p['port']}): {error}",
                throttle_duration_sec=10.0)

    def _close(self, reason: str) -> None:
        if self.ser is not None:
            self.get_logger().error(f'[Base] serial closed: {reason}')
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
        self.ser = None

    def _write(self, body: str) -> bool:
        if self.ser is None:
            return False
        try:
            self.ser.write(protocol.frame(body, self.p['use_checksum']))
            return True
        except (serial.SerialException, OSError) as error:
            self.tx_fail += 1
            self._close(f'write: {error}')
            return False

    def _io_tick(self) -> None:
        now = time.monotonic()
        if self.ser is None:
            self._open(now)
            self._update_link_status(now)
            return
        try:
            waiting = self.ser.in_waiting
            data = self.ser.read(waiting) if waiting else b''
        except (serial.SerialException, OSError) as error:
            self._close(f'read: {error}')
            return
        if data:
            self.rx.extend(data)
            if len(self.rx) > 8192:
                del self.rx[:-1024]
            while b'\n' in self.rx:
                line, _, rest = bytes(self.rx).partition(b'\n')
                self.rx = bytearray(rest)
                self._handle_line(line, now)
        self._update_link_status(now)

    # ------------------------------------------------------------------
    # 수신 줄 처리
    # ------------------------------------------------------------------
    def _handle_line(self, raw: bytes, now: float) -> None:
        parsed = protocol.parse_line(raw)
        if isinstance(parsed, protocol.Feedback):
            self._on_feedback(parsed, now)
        elif isinstance(parsed, protocol.EStop):
            if parsed.active != self.stm32_estop:
                self.get_logger().warning(f'[Base] STM32 E-stop: {int(parsed.active)}')
            self.stm32_estop = parsed.active
            self.last_est_mono = now
        elif isinstance(parsed, protocol.ImuSample):
            self._publish_imu(parsed)
        elif isinstance(parsed, protocol.Text):
            if self.p['passthrough_control']:
                self.status_pub.publish(String(data=parsed.text))
            self.get_logger().info(f'[Base] RX: {parsed.text}')
        elif isinstance(parsed, protocol.Bad) and parsed.reason != 'empty':
            self.bad_count += 1
            self.get_logger().warning(
                f'[Base] bad line ({parsed.reason}): {parsed.line[:60]!r}',
                throttle_duration_sec=2.0)

    def _on_feedback(self, fb: protocol.Feedback, now: float) -> None:
        speeds = [self.geometry.rpm_to_speed(r) for r in fb.rpms]
        vx, vy, wz, residual = swerve.forward(self.geometry, fb.angles, speeds)

        dt = 0.0
        if self.last_fb_mono is not None:
            host_dt = now - self.last_fb_mono
            stm_dt = (protocol.tick_delta_ms(self.last_fb_t_ms, fb.t_ms) / 1000.0
                      if self.last_fb_t_ms is not None else 0.0)
            # STM32 시계가 그럴듯하면 그것을, 아니면 수신 시각 차이를 쓴다.
            dt = stm_dt if 0.0 < stm_dt <= self.p['max_fb_gap_sec'] else host_dt
            if dt > self.p['max_fb_gap_sec']:
                dt = 0.0  # 긴 공백은 적분하지 않는다

        if dt > 0.0:
            yaw = self.pose[2] + 0.5 * wz * dt
            self.pose[0] += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
            self.pose[1] += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
            self.pose[2] = swerve.wrap_angle(self.pose[2] + wz * dt)

        self.last_fb_mono = now
        self.last_fb_t_ms = fb.t_ms
        self.measured_angles = list(fb.angles)
        self.last_twist = (vx, vy, wz)
        self.last_residual = residual
        self.fb_count += 1
        self._publish_odom(vx, vy, wz)

    def _publish_odom(self, vx: float, vy: float, wz: float) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p['odom_frame']
        msg.child_frame_id = self.p['base_frame']
        msg.pose.pose.position.x = self.pose[0]
        msg.pose.pose.position.y = self.pose[1]
        msg.pose.pose.orientation.z = math.sin(self.pose[2] / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.pose[2] / 2.0)
        px, py, pyaw = self.p['pose_variance']
        msg.pose.covariance = _diag6(px, py, BIG_VARIANCE, BIG_VARIANCE, BIG_VARIANCE, pyaw)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz
        tx, ty, tyaw = self.p['twist_variance']
        msg.twist.covariance = _diag6(tx, ty, BIG_VARIANCE, BIG_VARIANCE, BIG_VARIANCE, tyaw)
        self.odom_pub.publish(msg)

    def _publish_imu(self, sample: protocol.ImuSample) -> None:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p['imu_frame']
        msg.orientation_covariance[0] = -1.0  # 자세 추정값 없음
        (msg.angular_velocity.x, msg.angular_velocity.y,
         msg.angular_velocity.z) = sample.gyro
        (msg.linear_acceleration.x, msg.linear_acceleration.y,
         msg.linear_acceleration.z) = sample.accel
        gyro = self.p['imu_gyro_variance']
        accel = self.p['imu_accel_variance']
        msg.angular_velocity_covariance = [gyro, 0.0, 0.0, 0.0, gyro, 0.0, 0.0, 0.0, gyro]
        msg.linear_acceleration_covariance = [
            accel, 0.0, 0.0, 0.0, accel, 0.0, 0.0, 0.0, accel]
        self.imu_pub.publish(msg)

    # ------------------------------------------------------------------
    # 링크 / E-stop
    # ------------------------------------------------------------------
    def _link_ok(self, now: float) -> bool:
        return (
            self.ser is not None
            and self.last_fb_mono is not None
            and now - self.last_fb_mono <= self.p['fb_timeout_sec']
        )

    def _update_link_status(self, now: float) -> None:
        if self._link_ok(now):
            if not self.ever_linked:
                self.get_logger().info('[Base] STM32 feedback 수신 시작')
            self.ever_linked = True
            if self.reported != 'COMM_OK':
                self._report('COMM_OK')
            return
        if self.ever_linked or now - self.start_mono > self.p['startup_grace_sec']:
            if self.reported != 'COMM_FAILED':
                self.get_logger().error(
                    '[Base] STM32 feedback 없음/끊김 -> 바퀴 정지 명령, /estop_state=true')
                self._report('COMM_FAILED')

    def _report(self, status: str) -> None:
        self.reported = status
        self.bridge_pub.publish(String(data=status))

    def _estop_reason(self, now: float) -> str:
        """E-stop 으로 봐야 하는 이유. 해제면 ''."""
        if self.estop_latched:
            return 'latched_by_control_command'
        if not self._link_ok(now):
            return 'no_stm32_feedback'
        if self.p['estop_source'] == 'link':
            return ''
        if self.last_est_mono is None or now - self.last_est_mono > self.p['estop_timeout_sec']:
            return 'no_est_line'
        return 'stm32_estop' if self.stm32_estop else ''

    def _estop_tick(self) -> None:
        self.estop_pub.publish(Bool(data=bool(self._estop_reason(time.monotonic()))))
        # /bridge_status 는 바뀔 때만 내므로 늦게 붙은 구독자(팀원 상태기계)에게 현재 상태를 한 번 더 보낸다.
        count = self.bridge_pub.get_subscription_count()
        if count > self.bridge_subscribers and self.reported:
            self.bridge_pub.publish(String(data=self.reported))
        self.bridge_subscribers = count

    # ------------------------------------------------------------------
    # 명령
    # ------------------------------------------------------------------
    def _on_cmd(self, msg: Twist) -> None:
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if all(math.isfinite(v) for v in values):
            self.cmd = values
            self.cmd_mono = time.monotonic()

    def _on_control_command(self, msg: String) -> None:
        text = msg.data.strip()
        if not text or len(text) > 64 or any(c in text for c in '\r\n*'):
            self.get_logger().warning(f'[Base] control_command 거부: {msg.data!r}')
            return
        if text.upper() == 'ESTOP':
            self.estop_latched = True
            self.get_logger().error('[Base] ESTOP 수신: /control_reset RESET 전까지 바퀴 정지')
        sent = self._write(text)
        self.get_logger().info(f"[Base] TX: {text}{'' if sent else ' (링크 없음, 보내지 못함)'}")

    def _on_control_reset(self, msg: String) -> None:
        if msg.data.strip().upper() != 'RESET':
            return
        # STM32 도 ESTOP 을 래치하므로 RESET 을 함께 보낸다(docs/stm32_drive_protocol.md).
        sent = self._write('RESET')
        if self.estop_latched:
            self.estop_latched = False
            self.get_logger().info(
                f"[Base] RESET: ESTOP 래치 해제{'' if sent else ' (STM32 링크 없음)'}")

    def _current_angles(self, now: float) -> List[float]:
        if (self.p['steer_feedback'] and self.measured_angles is not None
                and self._link_ok(now)):
            return self.measured_angles
        return self.model_angles

    def _advance_model(self, targets: List[float], dt: float) -> None:
        step = math.radians(self.p['steer_rate_dps']) * dt
        for index, target in enumerate(targets):
            error = swerve.wrap_angle(target - self.model_angles[index])
            self.model_angles[index] += max(-step, min(step, error))

    def _control_tick(self) -> None:
        now = time.monotonic()
        dt = min(0.2, max(0.0, now - self.last_control_mono))
        self.last_control_mono = now

        twist = self.cmd if now - self.cmd_mono <= self.p['cmd_timeout_sec'] else ZERO_TWIST
        if self._estop_reason(now):
            twist = ZERO_TWIST
        self.seq = (self.seq + 1) % 65536

        if self.p['command_mode'] == 'body':
            self.last_used, self.last_gate, self.last_adjusted = twist, 1.0, ''
            self._write(protocol.format_vel(self.seq, *twist))
            return

        current = self._current_angles(now)
        result = swerve.inverse(self.geometry, self.limits, *twist, current)
        targets = [wheel.angle for wheel in result.wheels]
        gate = swerve.steering_gate(targets, current, self.steer_gate)
        rpms = [self.geometry.speed_to_rpm(wheel.speed * gate) for wheel in result.wheels]
        if result.adjusted:
            self.adjusted_count += 1
            self.get_logger().warning(
                f'[Base] 명령 조정({result.adjusted}): '
                f'{_fmt(twist)} -> {_fmt(result.used)}', throttle_duration_sec=2.0)
        self.last_used, self.last_gate, self.last_adjusted = result.used, gate, result.adjusted
        self._write(protocol.format_drv(self.seq, targets, rpms))
        self._advance_model(targets, dt)

    def send_stop(self) -> None:
        """종료 직전에 정지 명령을 한 번 보낸다(펌웨어 워치독보다 먼저 멈추도록)."""
        if self.p['command_mode'] == 'body':
            self._write(protocol.format_vel(self.seq + 1, *ZERO_TWIST))
        else:
            angles = self._current_angles(time.monotonic())
            self._write(protocol.format_drv(self.seq + 1, angles, [0.0] * 4))

    # ------------------------------------------------------------------
    # 상태 로그
    # ------------------------------------------------------------------
    def _heartbeat(self) -> None:
        now = time.monotonic()
        span = max(1e-6, now - self.window_start)
        fb_rate = self.fb_count / span
        self.fb_count = 0
        self.window_start = now
        cmd_age = min(now - self.cmd_mono, 99.0)
        reason = self._estop_reason(now) or 'clear'
        link = 'OK' if self._link_ok(now) else 'DOWN'
        self.get_logger().info(
            f'[Base] link={link} fb={fb_rate:.1f}Hz '
            f'bad={self.bad_count} tx_fail={self.tx_fail} estop={reason} '
            f'cmd={_fmt(self.cmd)} age={cmd_age:.2f}s used={_fmt(self.last_used)} '
            f'gate={self.last_gate:.2f} adjusted={self.adjusted_count} '
            f'odom=({self.pose[0]:.2f},{self.pose[1]:.2f},{math.degrees(self.pose[2]):.0f}deg) '
            f'meas={_fmt(self.last_twist)} residual={self.last_residual:.3f}')

    def destroy_node(self) -> None:
        try:
            self.send_stop()
        finally:
            self._close('shutdown')
            super().destroy_node()


def _diag6(*values: float) -> List[float]:
    matrix = [0.0] * 36
    for index, value in enumerate(values):
        matrix[index * 7] = float(value)
    return matrix


def _fmt(twist: Tuple[float, float, float]) -> str:
    return f'({twist[0]:.2f},{twist[1]:.2f},{twist[2]:.2f})'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = BaseBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
