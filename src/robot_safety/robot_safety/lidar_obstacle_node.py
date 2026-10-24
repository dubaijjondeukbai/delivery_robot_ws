#!/usr/bin/env python3
"""
배달로봇 전방 LiDAR 장애물 판단 노드.

입력
----
/scan                       sensor_msgs/msg/LaserScan

출력
----
/safety/obstacle_detected    std_msgs/msg/Bool
/safety/front_distance       std_msgs/msg/Float32

판단
----
- 시작하거나 센서 상태가 불확실하면 STOP.
- 전방 기본 ±35도를 검사.
- 기본적으로 0.8m 이내 점이 하나라도 있으면 STOP.
- 1.0m 이내 점이 없는 정상 스캔이 3회 연속 들어오면 CLEAR.
- 스캔이 끊기거나 오래된 데이터이면 STOP.
- 정상 상태에서도 안전 상태를 주기적으로 발행.

거리 출력
---------
유효한 거리: 전방에서 가장 가까운 측정값.
NaN: 데이터 이상 또는 센서 상태 불확실.
+inf: inf_is_clear=True이고 전방 유효 데이터가 모두 +inf인 경우.

주의
----
거리는 차체 앞면이 아니라 LiDAR 원점 기준이다.
전방 부채꼴만 검사하므로 후방 및 회전 중 차체 측면은 보장하지 않는다.
설정은 시작 시 읽으며, 변경 후에는 노드를 재시작한다.
"""

import math

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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class LidarObstacleNode(Node):
    """전방 장애물과 LiDAR 데이터의 유효성을 검사한다."""

    def __init__(self):
        super().__init__('lidar_obstacle_node')

        # --------------------------------------------------
        # 파라미터
        # 거리: m / 각도: scan 프레임 기준 degree / 시간: 초
        # --------------------------------------------------
        defaults = {
            'front_angle_deg': 35.0,
            'front_center_deg': 0.0,
            'stop_distance': 0.8,
            'release_distance': 1.0,
            'min_points': 1,
            'clear_scans_required': 3,
            'min_valid_fraction': 0.8,
            'scan_timeout': 0.5,
            'max_scan_age_sec': 0.5,
            'future_tolerance_sec': 0.1,
            'state_publish_rate_hz': 20.0,
            'inf_is_clear': False,
            'expected_scan_frame': 'laser_link',
            'scan_topic': '/scan',
            'obstacle_topic': '/safety/obstacle_detected',
            'distance_topic': '/safety/front_distance',
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.front_angle_deg = self._number('front_angle_deg')
        self.front_center_deg = self._number(
            'front_center_deg',
            signed=True,
        )
        self.stop_distance = self._number('stop_distance')
        self.release_distance = self._number('release_distance')
        self.min_points = self._integer('min_points')
        self.clear_scans_required = self._integer(
            'clear_scans_required'
        )
        self.min_valid_fraction = self._number(
            'min_valid_fraction'
        )
        self.scan_timeout = self._number('scan_timeout')
        self.max_scan_age_sec = self._number('max_scan_age_sec')
        self.future_tolerance_sec = self._number(
            'future_tolerance_sec'
        )
        rate = self._number('state_publish_rate_hz')

        self.inf_is_clear = self.get_parameter(
            'inf_is_clear'
        ).value

        if not isinstance(self.inf_is_clear, bool):
            raise ValueError('inf_is_clear must be bool')

        self.expected_scan_frame = self._text(
            'expected_scan_frame'
        )

        if not 0.0 < self.front_angle_deg <= 90.0:
            raise ValueError(
                'front_angle_deg must be in (0, 90]'
            )

        if self.release_distance <= self.stop_distance:
            raise ValueError(
                'release_distance must exceed stop_distance'
            )

        if self.min_valid_fraction > 1.0:
            raise ValueError(
                'min_valid_fraction must be <= 1'
            )

        self._half_angle = math.radians(self.front_angle_deg)
        self._center_angle = math.radians(self.front_center_deg)

        # timeout에는 시뮬레이션 시간과 독립적인 시계를 사용한다.
        self._steady_clock = Clock(
            clock_type=ClockType.STEADY_TIME
        )

        # --------------------------------------------------
        # 내부 상태
        # --------------------------------------------------
        self._last_valid_time = None
        self._last_valid_stamp = None
        self._last_ros_time = None
        self._clear_count = 0

        self.obstacle_detected = True
        self.front_distance = float('nan')
        self._reason = None

        # --------------------------------------------------
        # QoS
        # 스캔: 최신 데이터 1개, Best Effort
        # 상태: 최신 상태 1개, Reliable
        # --------------------------------------------------
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.obstacle_pub = self.create_publisher(
            Bool,
            self._text('obstacle_topic'),
            state_qos,
        )

        self.distance_pub = self.create_publisher(
            Float32,
            self._text('distance_topic'),
            state_qos,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self._text('scan_topic'),
            self.scan_callback,
            scan_qos,
        )

        self.timeout_timer = self.create_timer(
            1.0 / rate,
            self.check_scan_timeout,
            clock=self._steady_clock,
        )

        self._trip('waiting_for_scan')
        self.publish_state()

    # ======================================================
    # 파라미터 검증
    # ======================================================
    def _number(self, name, signed=False):
        value = self.get_parameter(name).value

        if isinstance(value, bool) or not isinstance(
            value, (int, float)
        ):
            raise ValueError(f'{name} must be numeric')

        value = float(value)

        if not math.isfinite(value) or (
            not signed and value <= 0.0
        ):
            raise ValueError(f'Invalid {name}: {value}')

        return value

    def _integer(self, name):
        value = self.get_parameter(name).value

        if type(value) is not int or value < 1:
            raise ValueError(
                f'{name} must be a positive integer'
            )

        return value

    def _text(self, name):
        value = self.get_parameter(name).value

        if not isinstance(value, str) or not value.strip():
            raise ValueError(
                f'{name} must be a non-empty string'
            )

        return value.strip()

    # ======================================================
    # 시간 및 상태 관리
    # ======================================================
    def _now(self):
        return self._steady_clock.now().nanoseconds * 1e-9

    def _set_state(self, stopped, reason):
        self.obstacle_detected = stopped

        # 같은 상태의 로그를 매 주기 반복하지 않는다.
        if reason != self._reason:
            log = self.get_logger()

            if stopped:
                log.warning(f'[LiDAR] STOP: {reason}')
            else:
                log.info('[LiDAR] CLEAR: front sector clear')

            self._reason = reason

    def _trip(self, reason):
        """데이터 이상 시 즉시 정지하고 해제 확인을 초기화한다."""
        self._clear_count = 0
        self._last_valid_time = None
        self.front_distance = float('nan')
        self._set_state(True, reason)

    # ======================================================
    # 스캔 구조 검사
    # ======================================================
    def _metadata_error(self, msg):
        if msg.header.frame_id != self.expected_scan_frame:
            return 'unexpected_scan_frame'

        if len(msg.ranges) < 2:
            return 'empty_or_short_scan'

        values = (
            msg.angle_min,
            msg.angle_max,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
        )

        if not all(math.isfinite(v) for v in values):
            return 'invalid_scan_metadata'

        if msg.angle_increment == 0.0:
            return 'zero_angle_increment'

        step = abs(msg.angle_increment)
        span = (len(msg.ranges) - 1) * step

        expected_end = msg.angle_min + (
            len(msg.ranges) - 1
        ) * msg.angle_increment

        if span > 2.0 * math.pi + step:
            return 'invalid_angular_span'

        if abs(expected_end - msg.angle_max) > max(step, 1e-6):
            return 'inconsistent_scan_angles'

        if not 0.0 <= msg.range_min < msg.range_max:
            return 'invalid_range_limits'

        if msg.range_max <= self.release_distance:
            return 'insufficient_measurement_range'

        if msg.range_min >= self.stop_distance:
            return 'stop_zone_outside_measurement_range'

        return ''

    # ======================================================
    # 스캔 촬영 시각 검사
    #
    # 메시지 timestamp는 ROS 시간과 비교한다.
    # 통신 timeout용 Steady 시간과 섞지 않는다.
    # ======================================================
    def _stamp_error(self, msg):
        ros_now = self.get_clock().now().nanoseconds

        previous_ros_time = self._last_ros_time
        self._last_ros_time = ros_now

        # 시뮬레이션 초기화 등으로 ROS 시간이 되돌아간 경우.
        if (
            previous_ros_time is not None
            and ros_now < previous_ros_time
        ):
            self._last_valid_stamp = None
            return 'ros_clock_moved_backwards'

        stamp = msg.header.stamp

        if stamp.sec < 0 or not (
            0 <= stamp.nanosec < 1000000000
        ):
            return 'invalid_scan_stamp'

        stamp_ns = (
            stamp.sec * 1000000000
            + stamp.nanosec
        )

        if ros_now <= 0 or stamp_ns <= 0:
            return 'waiting_for_valid_clock_and_stamp'

        age = (ros_now - stamp_ns) * 1e-9

        if age >= self.max_scan_age_sec:
            return 'stale_scan_stamp'

        if age < -self.future_tolerance_sec:
            return 'scan_stamp_in_future'

        # 같은 스캔을 반복해서 받아도 정상 연속 스캔으로 세지 않는다.
        if (
            self._last_valid_stamp is not None
            and stamp_ns <= self._last_valid_stamp
        ):
            return 'repeated_or_reversed_scan_stamp'

        return ''

    # ======================================================
    # 스캔 수신
    # ======================================================
    def scan_callback(self, msg):
        now = self._now()

        # 타이머가 지연됐더라도 통신 공백을 놓치지 않는다.
        if (
            self._last_valid_time is not None
            and now - self._last_valid_time >= self.scan_timeout
        ):
            self._trip('scan_timeout')

        error = (
            self._metadata_error(msg)
            or self._stamp_error(msg)
        )

        if error:
            self._trip(error)
            self.publish_state()
            return

        front_count = 0
        valid_count = 0
        close_count = 0
        release_count = 0
        nearest = float('inf')

        for index, distance in enumerate(msg.ranges):
            angle = (
                msg.angle_min
                + index * msg.angle_increment
                - self._center_angle
            )

            # -pi ~ +pi로 정규화한다.
            relative = math.atan2(
                math.sin(angle),
                math.cos(angle),
            )

            if abs(relative) > self._half_angle + 1e-9:
                continue

            front_count += 1

            # +inf가 정상적인 "반사체 없음"이라는 것이 확인된
            # 드라이버에서만 inf_is_clear=True를 사용한다.
            if (
                math.isinf(distance)
                and distance > 0
                and self.inf_is_clear
            ):
                valid_count += 1
                continue

            if not math.isfinite(distance):
                continue

            if not (
                msg.range_min <= distance <= msg.range_max
            ):
                continue

            valid_count += 1
            nearest = min(nearest, distance)

            close_count += int(
                distance <= self.stop_distance
            )
            release_count += int(
                distance <= self.release_distance
            )

        # 설정한 전방 각도 범위 대부분이 스캔에 포함돼야 한다.
        # 작은 일부 영역만 보고 전방 전체가 안전하다고 판단하지 않는다.
        required = max(
            self.min_points,
            3,
            math.ceil(
                0.9
                * 2.0
                * self._half_angle
                / abs(msg.angle_increment)
            ),
        )

        if front_count < required:
            self._trip('insufficient_front_coverage')

        elif valid_count < max(
            self.min_points,
            math.ceil(
                front_count * self.min_valid_fraction
            ),
        ):
            self._trip('insufficient_valid_front_ranges')

        else:
            self._last_valid_time = now

            stamp = msg.header.stamp
            self._last_valid_stamp = (
                stamp.sec * 1000000000
                + stamp.nanosec
            )

            self.front_distance = nearest

            # 정지는 지연 없이 적용한다.
            if close_count >= self.min_points:
                self._clear_count = 0
                self._set_state(
                    True,
                    'obstacle_in_stop_zone',
                )

            elif self.obstacle_detected:
                # 해제는 release_distance 안에 점이 하나도 없는
                # 정상 스캔이 연속으로 들어와야 한다.
                if release_count == 0:
                    self._clear_count += 1
                else:
                    self._clear_count = 0

                if (
                    self._clear_count
                    >= self.clear_scans_required
                ):
                    self._clear_count = 0
                    self._set_state(False, 'front_clear')

                else:
                    self._set_state(
                        True,
                        'waiting_for_clear_scans',
                    )

        self.publish_state()

    # ======================================================
    # 주기적 데이터 유효성 검사 및 상태 발행
    # ======================================================
    def check_scan_timeout(self):
        if self._last_valid_time is not None:
            steady_age = (
                self._now() - self._last_valid_time
            )

            ros_age = (
                self.get_clock().now().nanoseconds
                - self._last_valid_stamp
            ) * 1e-9

            if steady_age >= self.scan_timeout:
                self._trip('scan_timeout')

            elif ros_age >= self.max_scan_age_sec:
                self._trip('stale_scan_stamp')

            elif ros_age < -self.future_tolerance_sec:
                self._trip('ros_clock_changed')

        # 정상 상태에서도 계속 발행한다.
        # Safety Manager의 obstacle_timeout_sec와 연결된다.
        self.publish_state()

    def publish_state(self):
        self.obstacle_pub.publish(
            Bool(data=self.obstacle_detected)
        )

        self.distance_pub.publish(
            Float32(data=float(self.front_distance))
        )

    # ======================================================
    # 종료 처리
    # ======================================================
    def stop(self):
        self.timeout_timer.cancel()
        self._trip('shutdown')

        if self.context.ok():
            try:
                self.publish_state()
            except Exception:
                # 종료 중 통신 예외가 자원 해제를 막지 않도록 한다.
                pass


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = LidarObstacleNode()
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