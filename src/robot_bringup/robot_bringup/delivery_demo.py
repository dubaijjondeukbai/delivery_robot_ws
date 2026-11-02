"""
QR 배달 시연 노드 (로보월드): 출발 QR → Nav2 이동 → 정지 확인 → 도착 QR → ESP32 뚜껑 열기.

    ros2 launch robot_bringup delivery_demo.launch.py esp32_port:=/dev/ttyUSB0
    (robot.launch.py nav_mode:=navigation motion_enabled:=true 와
     robot_base base.launch.py 가 떠 있어야 함)

입력
    /vision/qr_result   qr_node 가 읽은 QR 문자열 (START:A, OPEN:A, CLOSE:A)
    /delivery/command   운영자 명령, QR 과 같은 형식 + CLOSE, CANCEL, RESET
    /esp32_status       ESP32 응답 (LID_OPENED, LID_OPEN_STEPS_DONE, LID_ERROR:..., LID_STATE:...)
    /stm32_status       STM32 응답 (DRIVE_STOPPED, DRIVE_MOVING) — robot_base base_bridge_node 가 넘김
    /odom               실제로 섰는지 확인 (바퀴 오도메트리 + EKF)
출력
    /navigate_to_pose   Nav2 목표 (지점 좌표는 stations.yaml, `stations record A` 로 저장)
    /control_command    STOP (도착 시 STM32 정지 요청, base_bridge_node 경유)
    /esp32_command      LID_OPEN / LID_CLOSE / LID_STOP (ESP32 브리지 경유)
    /delivery/state     상태 JSON (바뀔 때 + 1 초마다)
    /delivery/lid_closed  Bool 10 Hz, 뚜껑이 닫힘(확인 안 된 닫힘 포함)인지 → robot_safety 뚜껑 인터록

상태기계 규칙은 delivery_logic.py. ESP32 줄 형식은 docs/esp32_lid_protocol.md.
"""

from __future__ import annotations

import json
import math
import time
from typing import Dict, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robot_bringup import delivery_logic as logic
from robot_bringup.stations import load_stations, yaw_to_quaternion
from std_msgs.msg import Bool, String


class DeliveryDemo(Node):

    def __init__(self) -> None:
        super().__init__('delivery_demo')
        defaults = {
            'stations_file': '~/maps/stations.yaml',
            'qr_topic': '/vision/qr_result',
            'command_topic': '/delivery/command',
            'state_topic': '/delivery/state',
            'lid_closed_topic': '/delivery/lid_closed',
            'esp32_command_topic': '/esp32_command',
            'esp32_status_topic': '/esp32_status',
            'control_command_topic': '/control_command',
            'stm32_status_topic': '/stm32_status',
            'odom_topic': '/odom',
            'require_drive_stopped': True,
            'stop_hold_sec': 0.5,
            'stop_linear_tol': 0.01,
            'stop_angular_tol': 0.02,
            'odom_timeout_sec': 0.5,
            'motion_linear_tol': 0.05,
            'motion_hold_sec': 0.3,
            'stop_timeout_sec': 5.0,
            'lid_timeout_sec': 30.0,
            'require_confirmed_open': True,
            'require_confirmed_close': False,
            'assume_closed_at_start': True,
            'auto_close_sec': 0.0,
            'return_to': '',
            'start_cooldown_sec': 3.0,
            'publish_navigation_status': False,
        }
        self.p = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            self.p[name] = self.get_parameter(name).value

        self.poses: Dict[str, Dict[str, float]] = {}
        self._reload_stations()
        self.machine = logic.DeliveryMachine(logic.DeliveryConfig(
            stations=frozenset(self.poses),
            require_confirmed_open=self.p['require_confirmed_open'],
            require_confirmed_close=self.p['require_confirmed_close'],
            assume_closed_at_start=self.p['assume_closed_at_start'],
            stop_timeout_sec=self.p['stop_timeout_sec'],
            lid_timeout_sec=self.p['lid_timeout_sec'],
            auto_close_sec=self.p['auto_close_sec'],
            return_to=self.p['return_to'],
            start_cooldown_sec=self.p['start_cooldown_sec'],
        ), now=time.monotonic())

        self.control_pub = self.create_publisher(String, self.p['control_command_topic'], 10)
        self.esp32_pub = self.create_publisher(String, self.p['esp32_command_topic'], 10)
        self.state_pub = self.create_publisher(String, self.p['state_topic'], 10)
        self.lid_closed_pub = self.create_publisher(Bool, self.p['lid_closed_topic'], 10)
        self.nav_status_pub = self.create_publisher(String, '/navigation_status', 10)
        self.create_subscription(String, self.p['qr_topic'], self._on_text, 10)
        self.create_subscription(String, self.p['command_topic'], self._on_text, 10)
        self.create_subscription(String, self.p['esp32_status_topic'], self._on_esp32, 10)
        self.create_subscription(String, self.p['stm32_status_topic'], self._on_stm32, 10)
        self.create_subscription(Odometry, self.p['odom_topic'], self._on_odom, 10)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.goal_handle = None
        self.goal_seq = 0
        self.odom: Optional[tuple] = None   # (vx, wz, 받은 시각)
        self.stopped_since: Optional[float] = None
        self.moving_since: Optional[float] = None
        # STM32 의 마지막 주행 상태 줄(DRIVE_MOVING / DRIVE_STOPPED)과 시각
        self.drive_status: Optional[str] = None
        self.drive_status_at = 0.0
        self.drive_moving_at: Optional[float] = None
        self.goal_sent_at: Optional[float] = None
        self.stop_sent_at: Optional[float] = None
        self.last_published = ''
        self.last_heartbeat = 0.0

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"[Delivery] 지점 {sorted(self.poses)} ({self.p['stations_file']}), "
            f"require_drive_stopped={self.p['require_drive_stopped']}, "
            f"require_confirmed_open={self.p['require_confirmed_open']}, "
            f"return_to={self.p['return_to'] or '-'}")
        if not self.poses:
            self.get_logger().warning(
                '[Delivery] 저장된 지점이 없습니다: stations record A 로 먼저 저장하세요')
        self._publish_state(force=True)

    # ------------------------------------------------------------------
    def _reload_stations(self) -> None:
        try:
            self.poses = load_stations(self.p['stations_file'])
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.get_logger().error(f'[Delivery] stations 파일 읽기 실패: {error}')
        if hasattr(self, 'machine'):
            self.machine.config.stations = frozenset(self.poses)

    def _run(self, actions) -> None:
        for action in actions:
            kind = action[0]
            if kind == 'goal':
                self._send_goal(action[1])
            elif kind == 'cancel':
                self._cancel_goal()
            elif kind == 'stm32':
                self.stop_sent_at = time.monotonic()
                self.stopped_since = None
                self.control_pub.publish(String(data=action[1]))
                self.get_logger().info(f'[Delivery] STM32 <- {action[1]}')
            elif kind == 'esp32':
                self.esp32_pub.publish(String(data=action[1]))
                self.get_logger().info(f'[Delivery] ESP32 <- {action[1]}')
            elif kind == 'event':
                self._log(action[1], f'[Delivery] {action[2]}')
        self._publish_state()

    def _log(self, level: str, text: str) -> None:
        # rclpy 는 같은 호출 위치에서 심각도를 바꾸면 예외를 낸다 → 수준마다 다른 줄에서 부른다.
        if level == 'error':
            self.get_logger().error(text)
        elif level == 'warn':
            self.get_logger().warning(text)
        elif level == 'info':
            self.get_logger().info(text)
        else:
            self.get_logger().debug(text)

    # ------------------------------------------------------------------
    def _on_text(self, msg: String) -> None:
        text = msg.data.strip()
        parsed = logic.parse_command(text)
        if parsed is not None and parsed[0] == 'START':
            self._reload_stations()  # 시연 중에 지점을 새로 저장했을 수 있다
        self._run(self.machine.on_text(text, time.monotonic()))

    def _on_esp32(self, msg: String) -> None:
        if 'LID' in msg.data.upper():
            self._run(self.machine.on_lid_line(msg.data, time.monotonic()))

    def _on_stm32(self, msg: String) -> None:
        text = msg.data.strip().upper()
        if text in ('DRIVE_STOPPED', 'DRIVE_MOVING'):
            self.drive_status = text
            self.drive_status_at = time.monotonic()
            if text == 'DRIVE_MOVING':
                self.drive_moving_at = self.drive_status_at

    def _drive_stop_confirmed(self) -> bool:
        """
        STM32 도 섰다고 했는지.

        펌웨어에 따라 DRIVE_STOPPED 를 STOP 의 응답으로 보내거나(팀원 mock),
        멈춘 순간에 한 번만 보낸다(이미 서 있으면 STOP 뒤에 다시 안 올 수 있음). 둘 다 받아들인다:
        STOP 뒤에 온 DRIVE_STOPPED, 또는 이번 이동에서 DRIVE_MOVING 다음에 온 DRIVE_STOPPED.
        """
        if not self.p['require_drive_stopped']:
            return True
        if self.drive_status != 'DRIVE_STOPPED':
            return False
        after_stop = self.stop_sent_at is not None and self.drive_status_at >= self.stop_sent_at
        moved_then_stopped = (self.drive_moving_at is not None and self.goal_sent_at is not None
                              and self.drive_moving_at >= self.goal_sent_at)
        return after_stop or moved_then_stopped

    def _on_odom(self, msg: Odometry) -> None:
        twist = msg.twist.twist
        self.odom = (twist.linear.x, twist.angular.z, time.monotonic())

    # ------------------------------------------------------------------
    def _send_goal(self, name: str) -> None:
        pose = self.poses.get(name)
        if pose is None:
            self._run(self.machine.on_nav_result(False, f'지점 {name} 좌표 없음', time.monotonic()))
            return
        if not self.client.wait_for_server(timeout_sec=2.0):
            self._run(self.machine.on_nav_result(
                False, '/navigate_to_pose 없음 (nav_mode:=navigation?)', time.monotonic()))
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = pose['x']
        goal.pose.pose.position.y = pose['y']
        (goal.pose.pose.orientation.x, goal.pose.pose.orientation.y,
         goal.pose.pose.orientation.z, goal.pose.pose.orientation.w) = yaw_to_quaternion(
            pose['yaw'])
        self.goal_seq += 1
        seq = self.goal_seq
        self.goal_sent_at = time.monotonic()
        self.get_logger().info(
            f"[Delivery] Nav2 목표 {name}: x={pose['x']:.2f} y={pose['y']:.2f} "
            f"yaw={math.degrees(pose['yaw']):.0f}deg")
        if self.p['publish_navigation_status']:
            self.nav_status_pub.publish(String(data='NAVIGATING'))
        future = self.client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_goal_response(f, seq))

    def _on_goal_response(self, future, seq: int) -> None:
        handle = future.result()
        if seq != self.goal_seq:
            return
        if handle is None or not handle.accepted:
            self._run(self.machine.on_nav_result(False, '목표 거부됨', time.monotonic()))
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_result(f, seq))

    def _on_result(self, future, seq: int) -> None:
        if seq != self.goal_seq:
            return  # 취소된 예전 목표
        self.goal_handle = None
        wrapped = future.result()
        status = wrapped.status if wrapped is not None else GoalStatus.STATUS_UNKNOWN
        ok = status == GoalStatus.STATUS_SUCCEEDED
        result = wrapped.result if wrapped is not None else None
        detail = f"status={status} {getattr(result, 'error_code', '')} " \
                 f"{getattr(result, 'error_msg', '')}".strip()
        if self.p['publish_navigation_status']:
            self.nav_status_pub.publish(String(data='GOAL_REACHED' if ok else 'FAILED'))
        self._run(self.machine.on_nav_result(ok, detail, time.monotonic()))

    def _cancel_goal(self) -> None:
        self.goal_seq += 1  # 이후 오는 결과는 무시
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self.get_logger().warning('[Delivery] Nav2 목표 취소 요청')

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        now = time.monotonic()
        before = self.machine.state
        actions = self.machine.tick(now)
        state = self.machine.state

        fresh = self.odom is not None and now - self.odom[2] <= self.p['odom_timeout_sec']
        still = fresh and abs(self.odom[0]) <= self.p['stop_linear_tol'] \
            and abs(self.odom[1]) <= self.p['stop_angular_tol']
        moving = fresh and abs(self.odom[0]) > self.p['motion_linear_tol']

        if before == logic.STOPPING and state == logic.ERROR:
            odom_text = 'odom 없음/늦음' if not fresh else \
                f'odom vx={self.odom[0]:.3f} wz={self.odom[1]:.3f}'
            self.get_logger().error(
                f'[Delivery] 정지 확인 실패: {odom_text}, STM32 마지막 상태='
                f"{self.drive_status or '받은 적 없음'} "
                f"(require_drive_stopped={self.p['require_drive_stopped']})")

        if state == logic.STOPPING:
            self.stopped_since = (self.stopped_since or now) if still else None
            held = self.stopped_since is not None and \
                now - self.stopped_since >= self.p['stop_hold_sec']
            if held and self._drive_stop_confirmed():
                actions += self.machine.on_stop_confirmed(now)

        if state in (logic.WAIT_OPEN, logic.OPENING, logic.OPEN, logic.OPEN_UNCONFIRMED,
                     logic.CLOSING):
            self.moving_since = (self.moving_since or now) if moving else None
            if self.moving_since is not None and \
                    now - self.moving_since >= self.p['motion_hold_sec']:
                self.moving_since = None
                actions += self.machine.on_unexpected_motion(now)
        else:
            self.moving_since = None

        if actions:
            self._run(actions)
        elif now - self.last_heartbeat >= 1.0:
            self._publish_state(force=True)
        # 뚜껑 인터록(robot_safety): 닫힘(확인 안 된 닫힘 포함)일 때만 True. 10 Hz 로 계속 낸다.
        self.lid_closed_pub.publish(Bool(data=self.machine.lid in (
            logic.LID_CLOSED, logic.LID_CLOSED_UNCONFIRMED)))

    def _publish_state(self, force: bool = False) -> None:
        snapshot = self.machine.snapshot()
        text = json.dumps(snapshot, ensure_ascii=False)
        if force or text != self.last_published:
            self.state_pub.publish(String(data=text))
            self.last_published = text
            self.last_heartbeat = time.monotonic()

    def shutdown(self) -> None:
        """종료 직전: 이동 중이면 목표 취소, 뚜껑이 움직이는 중이면 LID_STOP."""
        try:
            if self.machine.state in (logic.NAVIGATING, logic.RETURNING):
                self._cancel_goal()
            if self.machine.state in (logic.OPENING, logic.CLOSING):
                self.esp32_pub.publish(String(data='LID_STOP'))
        except Exception as error:  # 종료 중 컨텍스트가 이미 닫혔을 수 있다
            self.get_logger().warning(f'[Delivery] 종료 처리 실패: {error}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DeliveryDemo()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
