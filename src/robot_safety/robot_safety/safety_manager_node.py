#!/usr/bin/env python3
"""
배달로봇 최종 속도 안전 관리 노드.

입력
----
/cmd_vel_auto              geometry_msgs/msg/Twist
/estop_state               std_msgs/msg/Bool
/safety/obstacle_detected   std_msgs/msg/Bool
/vision/person_hazard      std_msgs/msg/Bool   (use_vision_hazard=true 일 때만, 기본 false)
/delivery/lid_closed       std_msgs/msg/Bool   (use_lid_interlock=true 일 때만, 기본 false)
/safety/rear_obstacle_detected  std_msgs/msg/Bool   (allow_reverse_escape=true 일 때만, 기본 false)

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
7. 후진은 기본 금지한다(max_reverse_velocity 0). allow_reverse_escape=true 일 때만
   후방 LiDAR 영역(두 번째 lidar_obstacle_node, 뒤쪽 섹터)이 비어 있는 동안 느린 직선 후진을 허용한다.
   후진 명령에는 전방 장애물 정지를 적용하지 않고(벽 앞에서 빠져나오기 위함) 후방 장애물 정지를
   적용하며, 회전(angular.z)은 0 으로 만든다(앞바퀴 조향 차가 후진하며 꺾으면 앞이 옆으로 쓸린다).
8. 비전 위험 입력은 선택(use_vision_hazard). 켜면 정지 사유만 추가하며 LiDAR/E-stop 판단을
   대신하지 않는다. True 를 받는 즉시 정지하고, 입력이 끊기면 정지한다(모름 = 정지).
9. 뚜껑 인터록은 선택(use_lid_interlock). 켜면 뚜껑이 닫힘(True)이 아닐 때와 입력이 끊겼을 때
   정지한다. 열린 뚜껑이 LiDAR 뒤쪽을 가리고 짐이 떨어질 수 있으므로 열린 채 주행하지 않는다.

통신 조건
---------
E-stop과 장애물 상태는 상태 변경 시뿐 아니라 주기적으로 발행해야 한다.
정상 상태에서도 Bool(data=False)를 계속 발행해야 한다.

모든 설정은 노드 시작 시 읽는다.
설정을 변경하려면 파라미터 파일을 수정하고 노드를 재시작한다.
"""

import math

from geometry_msgs.msg import Twist
import rclpy
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
        'waiting_for_vision_hazard',
        'vision_hazard_timeout',
        'vision_hazard',
        'waiting_for_lid_state',
        'lid_state_timeout',
        'lid_open',
        'waiting_for_rear_obstacle_state',
        'rear_obstacle_state_timeout',
        'rear_obstacle_detected',
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
            # 선택: 비전 사람 위험 입력(robot_vision hazard_monitor_node). 기본 사용 안 함.
            'use_vision_hazard': False,
            'vision_hazard_topic': '/vision/person_hazard',
            'vision_hazard_timeout_sec': 0.5,
            # 선택: 뚜껑 인터록(robot_bringup delivery_demo 가 발행). 기본 사용 안 함.
            'use_lid_interlock': False,
            'lid_closed_topic': '/delivery/lid_closed',
            'lid_state_timeout_sec': 1.0,
            # 선택: 막혔을 때 느린 후진 탈출(Nav2 BackUp). 기본 사용 안 함.
            'allow_reverse_escape': False,
            'rear_obstacle_topic': '/safety/rear_obstacle_detected',
            'rear_obstacle_timeout_sec': 0.5,
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

        self.use_vision_hazard = self.get_parameter('use_vision_hazard').value
        if not isinstance(self.use_vision_hazard, bool):
            raise ValueError('use_vision_hazard must be a bool')
        self.vision_hazard_timeout_sec = self._number_parameter(
            'vision_hazard_timeout_sec'
        )
        vision_topic = self._topic_parameter('vision_hazard_topic')

        self.use_lid_interlock = self.get_parameter('use_lid_interlock').value
        if not isinstance(self.use_lid_interlock, bool):
            raise ValueError('use_lid_interlock must be a bool')
        self.lid_state_timeout_sec = self._number_parameter(
            'lid_state_timeout_sec'
        )
        lid_topic = self._topic_parameter('lid_closed_topic')

        self.allow_reverse_escape = self.get_parameter('allow_reverse_escape').value
        if not isinstance(self.allow_reverse_escape, bool):
            raise ValueError('allow_reverse_escape must be a bool')
        self.rear_obstacle_timeout_sec = self._number_parameter(
            'rear_obstacle_timeout_sec'
        )
        rear_topic = self._topic_parameter('rear_obstacle_topic')
        if self.allow_reverse_escape and self.max_reverse_velocity <= 0.0:
            raise ValueError(
                'allow_reverse_escape needs max_reverse_velocity > 0'
            )
        if not self.allow_reverse_escape:
            # 후방 판단 없이 후진하지 않는다(설정 파일의 값과 관계없이).
            self.max_reverse_velocity = 0.0

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
        self.vision_hazard = True
        self.lid_closed = False
        self.rear_obstacle_detected = True

        # None이면 아직 해당 입력을 받지 않은 상태다.
        self._last_estop_time = None
        self._last_obstacle_time = None
        self._last_vision_time = None
        self._last_lid_time = None
        self._last_rear_time = None
        self._last_cmd_time = None

        self._latest_auto_cmd = None
        self._last_cmd_reverse = False  # 마지막으로 받은 명령이 후진(탈출)인지
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

        # 비전 위험 입력은 켰을 때만 구독한다(끈 상태에서는 기존 동작과 같다).
        self.vision_sub = None
        if self.use_vision_hazard:
            self.vision_sub = self.create_subscription(
                Bool,
                vision_topic,
                self.vision_hazard_callback,
                input_qos,
            )

        # 뚜껑 인터록 입력도 켰을 때만 구독한다.
        self.lid_sub = None
        if self.use_lid_interlock:
            self.lid_sub = self.create_subscription(
                Bool,
                lid_topic,
                self.lid_closed_callback,
                input_qos,
            )

        # 후방 장애물 입력은 후진 탈출을 켰을 때만 구독한다.
        self.rear_sub = None
        if self.allow_reverse_escape:
            self.rear_sub = self.create_subscription(
                Bool,
                rear_topic,
                self.rear_obstacle_callback,
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
            f'estop_timeout={self.estop_timeout_sec:.2f} s | '
            f'vision_hazard='
            f'{vision_topic if self.use_vision_hazard else "off"} | '
            f'lid_interlock='
            f'{lid_topic if self.use_lid_interlock else "off"} | '
            f'reverse_escape='
            f'{rear_topic if self.allow_reverse_escape else "off"}'
            f' (max {self.max_reverse_velocity:.2f} m/s)'
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

    def _check_safety(self, now, reverse=None):
        """
        안전 입력을 검사한다.

        reverse 가 True 면 후진 명령 기준(전방 대신 후방 장애물), None 이면 마지막으로 받은
        명령의 방향 기준(저장된 명령이 있으면 그 명령의 방향과 같다).
        차단 상태이면 명령을 폐기하고 원인 문자열을 반환한다.
        안전하면 빈 문자열을 반환한다.
        """
        if reverse is None:
            reverse = self._last_cmd_reverse

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

        elif self.obstacle_detected and not reverse:
            reason = 'obstacle_detected'

        elif reverse and self._last_rear_time is None:
            reason = 'waiting_for_rear_obstacle_state'

        elif (
            reverse
            and now - self._last_rear_time >= self.rear_obstacle_timeout_sec
        ):
            reason = 'rear_obstacle_state_timeout'

        elif reverse and self.rear_obstacle_detected:
            reason = 'rear_obstacle_detected'

        elif self.use_vision_hazard and self._last_vision_time is None:
            reason = 'waiting_for_vision_hazard'

        elif (
            self.use_vision_hazard
            and now - self._last_vision_time >= self.vision_hazard_timeout_sec
        ):
            reason = 'vision_hazard_timeout'

        elif self.use_vision_hazard and self.vision_hazard:
            reason = 'vision_hazard'

        elif self.use_lid_interlock and self._last_lid_time is None:
            reason = 'waiting_for_lid_state'

        elif (
            self.use_lid_interlock
            and now - self._last_lid_time >= self.lid_state_timeout_sec
        ):
            reason = 'lid_state_timeout'

        elif self.use_lid_interlock and not self.lid_closed:
            reason = 'lid_open'

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

        # 후진 탈출 명령(allow_reverse_escape 일 때만): 전방 대신 후방 장애물로 판단한다.
        reverse = (
            self.allow_reverse_escape
            and math.isfinite(msg.linear.x)
            and msg.linear.x < -1e-3
        )
        self._last_cmd_reverse = reverse

        # 안전 입력이 오래됐거나 위험 상태라면 이동 명령을 저장하지 않는다.
        if self._check_safety(now, reverse):
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

        if reverse:
            # 느린 직선 후진만 (회전 0)
            safe_cmd.linear.x = max(
                -self.max_reverse_velocity,
                float(msg.linear.x),
            )
        else:
            safe_cmd.linear.x = max(
                0.0,
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
    # 비전 사람 위험 콜백 (use_vision_hazard=true 일 때만 구독)
    # ======================================================
    def vision_hazard_callback(self, msg):
        now = self._now()

        self._check_safety(now)

        self.vision_hazard = bool(msg.data)
        self._last_vision_time = now

        # True 를 받은 그 콜백에서 바로 정지 명령을 낸다(다음 타이머를 기다리지 않음).
        self.publish_safe_command()

    # ======================================================
    # 후방 장애물 콜백 (allow_reverse_escape=true 일 때만 구독)
    # ======================================================
    def rear_obstacle_callback(self, msg):
        now = self._now()

        self._check_safety(now)

        self.rear_obstacle_detected = bool(msg.data)
        self._last_rear_time = now

        # 후진 중 뒤에 장애물이 들어오면 그 콜백에서 바로 정지한다.
        self.publish_safe_command()

    # ======================================================
    # 뚜껑 닫힘 콜백 (use_lid_interlock=true 일 때만 구독)
    # ======================================================
    def lid_closed_callback(self, msg):
        now = self._now()

        self._check_safety(now)

        self.lid_closed = bool(msg.data)
        self._last_lid_time = now

        # 열림(False)을 받은 그 콜백에서 바로 정지 명령을 낸다.
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
