#!/usr/bin/env python3
"""
배달로봇 최종 속도 안전 관리 노드.

입력
----
/cmd_vel_auto              geometry_msgs/msg/Twist
/estop_state               std_msgs/msg/Bool
/safety/obstacle_detected   std_msgs/msg/Bool

출력
----
/cmd_vel_safe              geometry_msgs/msg/Twist
/safety/state              std_msgs/msg/String   READY 또는 STOP:<사유>

안전 원칙
---------
1. 필요한 안전 상태를 받기 전에는 정지한다.
2. 명령 또는 안전 상태가 오래되면 정지한다.
3. E-stop 또는 장애물 감지 시 정지한다.
4. 정지 중 받은 이동 명령은 저장하지 않는다.
5. 정지 해제 후에는 새 이동 명령을 기다린다.
6. linear.x와 angular.z만 사용한다.
7. 후방 안전 판단을 구현하기 전까지 후진은 기본 금지한다.

통신 조건
---------
E-stop과 장애물 상태는 상태 변경 시뿐 아니라 주기적으로 발행해야 한다.
정상 상태에서도 Bool(data=False)를 계속 발행해야 한다.

모든 설정은 노드 시작 시 읽는다.
설정을 변경하려면 파라미터 파일을 수정하고 노드를 재시작한다.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String


class SafetyManagerNode(Node):
    """안전 상태와 명령 유효성을 확인한 뒤 속도를 출력한다."""

    # _check_safety 가 돌려주는 차단 사유. 명령이 없어서 서 있는
    # 정상 대기(waiting_for_new_command 등)와 달리 주기적으로 다시 알린다.
    SAFETY_BLOCK_REASONS = frozenset((
        'waiting_for_estop_state',
        'estop_state_timeout',
        'estop_active',
        'waiting_for_obstacle_state',
        'obstacle_state_timeout',
        'obstacle_detected',
    ))

    def __init__(self):
        super().__init__('safety_manager_node')

        # --------------------------------------------------
        # 파라미터 선언
        # 시간 단위: 초 / 속도 단위: m/s, rad/s
        # --------------------------------------------------
        defaults = {
            'control_rate_hz': 20.0,
            'cmd_timeout_sec': 0.5,
            'obstacle_timeout_sec': 0.5,
            'estop_timeout_sec': 1.0,
            'max_linear_velocity': 0.3,
            'max_reverse_velocity': 0.0,
            'max_angular_velocity': 0.5,
            'cmd_vel_auto_topic': '/cmd_vel_auto',
            'cmd_vel_safe_topic': '/cmd_vel_safe',
            'estop_topic': '/estop_state',
            'obstacle_topic': '/safety/obstacle_detected',
            'state_topic': '/safety/state',
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        # 잘못된 설정을 조용히 무시하지 않고 시작 단계에서 거부한다.
        self.control_rate_hz = self._number_parameter(
            'control_rate_hz'
        )
        self.cmd_timeout_sec = self._number_parameter(
            'cmd_timeout_sec'
        )
        self.obstacle_timeout_sec = self._number_parameter(
            'obstacle_timeout_sec'
        )
        self.estop_timeout_sec = self._number_parameter(
            'estop_timeout_sec'
        )
        self.max_linear_velocity = self._number_parameter(
            'max_linear_velocity',
            allow_zero=True,
        )
        self.max_reverse_velocity = self._number_parameter(
            'max_reverse_velocity',
            allow_zero=True,
        )
        self.max_angular_velocity = self._number_parameter(
            'max_angular_velocity',
            allow_zero=True,
        )

        cmd_topic = self._topic_parameter('cmd_vel_auto_topic')
        safe_topic = self._topic_parameter('cmd_vel_safe_topic')
        estop_topic = self._topic_parameter('estop_topic')
        obstacle_topic = self._topic_parameter('obstacle_topic')
        state_topic = self._topic_parameter('state_topic')

        # 입력과 출력이 같은 토픽으로 연결되는 피드백을 방지한다.
        if self.resolve_topic_name(cmd_topic) == (
            self.resolve_topic_name(safe_topic)
        ):
            raise ValueError(
                'cmd_vel_auto_topic and cmd_vel_safe_topic '
                'must resolve to different topics'
            )

        # --------------------------------------------------
        # 안전 판단용 시계
        #
        # use_sim_time이나 시스템 시각 변경의 영향을 받지 않는다.
        # 타이머와 데이터 수신 시각 모두 같은 시계를 사용한다.
        # --------------------------------------------------
        self._steady_clock = Clock(
            clock_type=ClockType.STEADY_TIME
        )

        # --------------------------------------------------
        # 내부 상태
        # --------------------------------------------------
        self.estop_active = True
        self.obstacle_detected = True

        # None이면 아직 해당 입력을 받지 않은 상태다.
        self._last_estop_time = None
        self._last_obstacle_time = None
        self._last_cmd_time = None

        self._latest_auto_cmd = None
        self._command_block_reason = 'waiting_for_new_command'
        self._last_output_reason = None

        # --------------------------------------------------
        # QoS
        #
        # 입력:
        #   Best Effort / Reliable 발행자 모두와 연결할 수 있다.
        #   통신 이상은 별도의 timeout으로 감시한다.
        #
        # 출력:
        #   Reliable, 최신 명령 1개만 유지한다.
        #
        # Volatile:
        #   과거 명령이나 안전 상태를 새 연결에 재전송하지 않는다.
        # --------------------------------------------------
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.safe_cmd_pub = self.create_publisher(
            Twist,
            safe_topic,
            output_qos,
        )

        # 현재 통과/차단 상태와 사유.
        # 로그는 상태가 바뀔 때만 남으므로, "왜 안 움직이는지"를
        # ros2 topic echo 로 언제든 볼 수 있게 매 출력마다 발행한다.
        self.state_pub = self.create_publisher(
            String,
            state_topic,
            output_qos,
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            cmd_topic,
            self.cmd_callback,
            input_qos,
        )

        self.estop_sub = self.create_subscription(
            Bool,
            estop_topic,
            self.estop_callback,
            input_qos,
        )

        self.obstacle_sub = self.create_subscription(
            Bool,
            obstacle_topic,
            self.obstacle_callback,
            input_qos,
        )

        # 기본 실행 방식에서는 모든 콜백이 순서대로 처리된다.
        self.control_timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.publish_safe_command,
            clock=self._steady_clock,
        )

        self.get_logger().info(
            '[Safety] Started | '
            f'output={safe_topic} | '
            f'rate={self.control_rate_hz:.1f} Hz | '
            f'cmd_timeout={self.cmd_timeout_sec:.2f} s | '
            f'obstacle_timeout={self.obstacle_timeout_sec:.2f} s | '
            f'estop_timeout={self.estop_timeout_sec:.2f} s'
        )

        # 시작 직후부터 정지 명령을 출력한다.
        self.publish_safe_command()

    # ======================================================
    # 파라미터 검증
    # ======================================================
    def _number_parameter(self, name, allow_zero=False):
        value = self.get_parameter(name).value

        if isinstance(value, bool) or not isinstance(
            value, (int, float)
        ):
            raise ValueError(f'{name} must be a number')

        value = float(value)

        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')

        if allow_zero:
            if value < 0.0:
                raise ValueError(f'{name} must be >= 0')
        elif value <= 0.0:
            raise ValueError(f'{name} must be > 0')

        return value

    def _topic_parameter(self, name):
        value = self.get_parameter(name).value

        if not isinstance(value, str) or not value.strip():
            raise ValueError(f'{name} must be a non-empty string')

        return value.strip()

    # ======================================================
    # 시간 및 명령 관리
    # ======================================================
    def _now(self):
        return self._steady_clock.now().nanoseconds * 1e-9

    def _clear_command(self, reason='waiting_for_new_command'):
        """저장된 명령을 폐기하고 새로운 명령을 기다린다."""
        self._latest_auto_cmd = None
        self._last_cmd_time = None
        self._command_block_reason = reason

    def _check_safety(self, now):
        """
        안전 입력을 검사한다.

        차단 상태이면 명령을 폐기하고 원인 문자열을 반환한다.
        안전하면 빈 문자열을 반환한다.
        """
        if self._last_estop_time is None:
            reason = 'waiting_for_estop_state'

        elif now - self._last_estop_time >= self.estop_timeout_sec:
            reason = 'estop_state_timeout'

        elif self.estop_active:
            reason = 'estop_active'

        elif self._last_obstacle_time is None:
            reason = 'waiting_for_obstacle_state'

        elif (
            now - self._last_obstacle_time
            >= self.obstacle_timeout_sec
        ):
            reason = 'obstacle_state_timeout'

        elif self.obstacle_detected:
            reason = 'obstacle_detected'

        else:
            reason = ''

        if reason:
            self._clear_command()

        return reason

    # ======================================================
    # 속도 명령 콜백
    # ======================================================
    def cmd_callback(self, msg):
        now = self._now()

        # 안전 입력이 오래됐거나 위험 상태라면 이동 명령을 저장하지 않는다.
        if self._check_safety(now):
            self.publish_safe_command()
            return

        values = (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )

        # NaN 또는 무한대가 포함된 명령은 전체 거부한다.
        if not all(math.isfinite(value) for value in values):
            self._clear_command('invalid_command_nonfinite')
            self.publish_safe_command()
            return

        # 이 로봇은 전후진과 평면 회전만 허용한다.
        unsupported_axes = (
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
        )

        if any(abs(value) > 1e-9 for value in unsupported_axes):
            self._clear_command('unsupported_command_axes')
            self.publish_safe_command()
            return

        # 원본 메시지와 분리된 새로운 Twist에 제한값을 적용한다.
        safe_cmd = Twist()

        safe_cmd.linear.x = max(
            -self.max_reverse_velocity,
            min(
                self.max_linear_velocity,
                float(msg.linear.x),
            ),
        )

        safe_cmd.angular.z = max(
            -self.max_angular_velocity,
            min(
                self.max_angular_velocity,
                float(msg.angular.z),
            ),
        )

        self._latest_auto_cmd = safe_cmd
        self._last_cmd_time = now
        self._command_block_reason = ''

        self.publish_safe_command()

    # ======================================================
    # E-stop 콜백
    # ======================================================
    def estop_callback(self, msg):
        now = self._now()

        # 새 메시지로 수신 시각을 갱신하기 전에 이전 상태를 검사한다.
        # 타이머 실행이 지연됐어도 통신 공백 중의 명령을 폐기한다.
        self._check_safety(now)

        self.estop_active = bool(msg.data)
        self._last_estop_time = now

        # E-stop 해제만으로 이전 이동 명령이 되살아나지 않는다.
        self.publish_safe_command()

    # ======================================================
    # LiDAR 장애물 상태 콜백
    # ======================================================
    def obstacle_callback(self, msg):
        now = self._now()

        self._check_safety(now)

        self.obstacle_detected = bool(msg.data)
        self._last_obstacle_time = now

        # 장애물 해제 후에도 새 이동 명령을 기다린다.
        self.publish_safe_command()

    # ======================================================
    # 최종 안전 출력
    # ======================================================
    def publish_safe_command(self):
        now = self._now()

        # 매번 정지 명령으로 시작한다.
        safe_cmd = Twist()
        reason = self._check_safety(now)

        if not reason:
            if (
                self._latest_auto_cmd is None
                or self._last_cmd_time is None
            ):
                reason = self._command_block_reason

            elif now - self._last_cmd_time >= self.cmd_timeout_sec:
                self._clear_command('command_timeout')
                reason = 'command_timeout'

            else:
                safe_cmd = self._latest_auto_cmd
                reason = 'command_valid'

        self.safe_cmd_pub.publish(safe_cmd)

        state = String()
        state.data = (
            'READY'
            if reason == 'command_valid'
            else f'STOP:{reason}'
        )
        self.state_pub.publish(state)

        # 같은 상태를 매 주기 출력하지 않고 상태 변경만 기록한다.
        if reason != self._last_output_reason:
            if reason == 'command_valid':
                self.get_logger().info(
                    '[Safety] READY: fresh command accepted'
                )
            else:
                self.get_logger().warning(
                    f'[Safety] STOP: {reason}'
                )

            self._last_output_reason = reason

        elif reason in self.SAFETY_BLOCK_REASONS:
            # 안전 입력 때문에 차단이 길어지면 5초마다 사유를 다시 알린다
            # (시작 직후의 'waiting_for_estop_state' 한 줄은 다른 로그에 묻히기 쉽다).
            self.get_logger().warning(
                f'[Safety] STOP continues: {reason}',
                throttle_duration_sec=5.0,
                throttle_time_source_type=self._steady_clock,
                skip_first=True,
            )

    # ======================================================
    # 종료 처리
    # ======================================================
    def stop(self):
        """
        정상 종료 시 가능한 경우 마지막 정지 명령을 발행한다.

        강제 종료나 이미 종료된 ROS 통신에서는 전달을 보장할 수 없다.
        """
        self.control_timer.cancel()
        self._clear_command('shutdown')

        if self.context.ok():
            try:
                self.safe_cmd_pub.publish(Twist())
            except Exception:
                # 통신 종료 중 발생한 예외로 자원 해제를 방해하지 않는다.
                pass


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SafetyManagerNode()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        try:
            if node is not None:
                try:
                    node.stop()
                finally:
                    node.destroy_node()
        finally:
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()