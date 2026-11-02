"""
전시·시연용 목표 지점 도구: 지점을 이름으로 저장하고, 그 지점으로 Nav2 를 보낸다.

    ros2 run robot_bringup stations where                 # 지금 위치(map 기준) 출력
    ros2 run robot_bringup stations record booth_a        # 지금 위치를 booth_a 로 저장
    ros2 run robot_bringup stations list
    ros2 run robot_bringup stations go booth_a            # booth_a 로 이동 (Ctrl+C = 취소)
    ros2 run robot_bringup stations tour booth_a booth_b --loop --dwell 5
    ros2 run robot_bringup stations remove booth_a

* 저장 파일: --file (기본 ~/maps/stations.yaml). 지점은 map 좌표(x, y, yaw)라 지도를 새로 만들면
  다시 저장해야 한다.
* 이동은 Nav2 의 /navigate_to_pose 액션을 쓴다(RViz 2D Goal Pose 와 같은 길).
  Pi 에서 SSH 로 실행하면 노트북-Pi 사이 DDS 연결 없이도 시연할 수 있다.
* 팀원 상태기계(delivery_robot_control)용 /navigation_status 를 함께 낸다:
  NAVIGATING(목표 수락·장애물 해제), OBSTACLE_STOP(Safety 가 장애물로 정지), GOAL_REACHED, FAILED.
  끄려면 --no-status.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
import time
from typing import Dict, List, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros
import yaml

DEFAULT_FILE = os.path.join('~', 'maps', 'stations.yaml')
NAME_PATTERN = re.compile(r'^[A-Za-z0-9_\-]{1,40}$')
OBSTACLE_REASONS = ('STOP:obstacle_detected', 'STOP:vision_hazard')
STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}


# ----------------------------------------------------------------------
# 순수 함수 (단위 시험 대상)
# ----------------------------------------------------------------------
def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def valid_name(name: str) -> bool:
    return bool(NAME_PATTERN.match(name))


def load_stations(path: str) -> Dict[str, Dict[str, float]]:
    path = os.path.expanduser(path)
    if not os.path.isfile(path):
        return {}
    with open(path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    stations = data.get('stations') or {}
    result = {}
    for name, pose in stations.items():
        result[str(name)] = {
            'x': float(pose['x']), 'y': float(pose['y']), 'yaw': float(pose['yaw'])}
    return result


def save_stations(path: str, stations: Dict[str, Dict[str, float]]) -> str:
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    data = {
        'frame_id': 'map',
        'stations': {
            name: {k: round(float(v), 4) for k, v in pose.items()}
            for name, pose in stations.items()
        },
    }
    temporary = path + '.tmp'
    with open(temporary, 'w', encoding='utf-8') as handle:
        handle.write('# robot_bringup stations (map 좌표, yaw = rad). 지도를 바꾸면 다시 저장\n')
        yaml.safe_dump(data, handle, sort_keys=False, allow_unicode=True)
    os.replace(temporary, path)
    return path


def status_for_safety(state: str, navigating: bool, obstacle_reported: bool) -> Optional[str]:
    """Safety 상태 변화를 팀원 상태기계용 문자열로 바꾼다. 보낼 것이 없으면 None."""
    if not navigating:
        return None
    blocked = state.startswith(OBSTACLE_REASONS)
    if blocked and not obstacle_reported:
        return 'OBSTACLE_STOP'
    if state == 'READY' and obstacle_reported:
        return 'NAVIGATING'
    return None


def strip_ros_args(argv: List[str]) -> List[str]:
    if '--ros-args' in argv:
        return argv[:argv.index('--ros-args')]
    return argv


# ----------------------------------------------------------------------
# ROS
# ----------------------------------------------------------------------
class StationClient(Node):

    def __init__(self, publish_status: bool) -> None:
        super().__init__('stations')
        self.publish_status = publish_status
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.status_pub = self.create_publisher(String, '/navigation_status', 10)
        self.create_subscription(String, '/safety/state', self._on_safety, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.navigating = False
        self.obstacle_reported = False
        self.safety_state = ''
        self.last_print = 0.0

    def _status(self, text: str) -> None:
        print(f'  -> /navigation_status {text}', flush=True)
        if self.publish_status:
            self.status_pub.publish(String(data=text))

    def _on_safety(self, msg: String) -> None:
        self.safety_state = msg.data
        update = status_for_safety(msg.data, self.navigating, self.obstacle_reported)
        if update is not None:
            self.obstacle_reported = update == 'OBSTACLE_STOP'
            print(f'  Safety: {msg.data}', flush=True)
            self._status(update)

    def current_pose(self, timeout: float = 5.0) -> Optional[Tuple[float, float, float]]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            return t.x, t.y, quaternion_to_yaw(q.x, q.y, q.z, q.w)
        return None

    def _on_feedback(self, message) -> None:
        now = time.monotonic()
        if now - self.last_print < 2.0:
            return
        self.last_print = now
        feedback = message.feedback
        print(
            f'  남은 거리 {feedback.distance_remaining:.2f} m, '
            f'경과 {feedback.navigation_time.sec} s, 복구 {feedback.number_of_recoveries}회'
            + (f', Safety={self.safety_state}' if self.safety_state else ''),
            flush=True)

    def go(self, name: str, pose: Dict[str, float]) -> bool:
        if not self.client.wait_for_server(timeout_sec=10.0):
            print('[stations] /navigate_to_pose 액션 서버 없음 (nav_mode:=navigation 으로 실행했나요?)')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = pose['x']
        goal.pose.pose.position.y = pose['y']
        (goal.pose.pose.orientation.x, goal.pose.pose.orientation.y,
         goal.pose.pose.orientation.z, goal.pose.pose.orientation.w) = yaw_to_quaternion(
            pose['yaw'])

        print(f"[stations] {name} 로 이동: x={pose['x']:.2f} y={pose['y']:.2f} "
              f"yaw={math.degrees(pose['yaw']):.0f}deg", flush=True)
        send = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            print('[stations] 목표 거부됨')
            self._status('FAILED')
            return False

        self.navigating = True
        self.obstacle_reported = False
        self._status('NAVIGATING')
        result_future = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future)
        except KeyboardInterrupt:
            print('\n[stations] Ctrl+C: Nav2 목표 취소 요청')
            cancel = handle.cancel_goal_async()
            deadline = time.monotonic() + 3.0
            while not cancel.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            accepted = cancel.done() and cancel.result() is not None and \
                len(cancel.result().goals_canceling) > 0
            print('[stations] 취소 ' + ('수락됨 (로봇 정지)' if accepted else
                                      '확인 안 됨 → RViz/E-stop 으로 멈추세요'))
            self.navigating = False
            self._status('FAILED')
            raise
        self.navigating = False

        wrapped = result_future.result()
        status = wrapped.status if wrapped is not None else GoalStatus.STATUS_UNKNOWN
        if status == GoalStatus.STATUS_SUCCEEDED:
            print(f'[stations] {name} 도착')
            self._status('GOAL_REACHED')
            return True
        result = wrapped.result if wrapped is not None else None
        code = getattr(result, 'error_code', '')
        text = getattr(result, 'error_msg', '')
        print(f'[stations] {name} 실패: {STATUS_NAMES.get(status, status)} {code} {text}'.rstrip())
        self._status('FAILED')
        return False


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog='stations', description='map 좌표 지점 저장/이동')
    parser.add_argument('--file', default=DEFAULT_FILE)
    parser.add_argument('--no-status', dest='status', action='store_false',
                        help='/navigation_status 를 내지 않는다')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('list')
    sub.add_parser('where')
    record = sub.add_parser('record')
    record.add_argument('name')
    remove = sub.add_parser('remove')
    remove.add_argument('name')
    go = sub.add_parser('go')
    go.add_argument('name')
    tour = sub.add_parser('tour')
    tour.add_argument('names', nargs='+')
    tour.add_argument('--loop', action='store_true')
    tour.add_argument('--dwell', type=float, default=3.0)
    tour.add_argument('--keep-going', action='store_true', help='실패해도 다음 지점으로')
    return parser.parse_args(strip_ros_args(argv))


def _print_list(stations: Dict[str, Dict[str, float]], path: str) -> None:
    print(f'{os.path.expanduser(path)}: {len(stations)}개')
    for name, pose in stations.items():
        print(f"  {name:<16} x={pose['x']:7.2f} y={pose['y']:7.2f} "
              f"yaw={math.degrees(pose['yaw']):6.1f}deg")


def run(args: argparse.Namespace) -> int:
    stations = load_stations(args.file)

    if args.command == 'list':
        _print_list(stations, args.file)
        return 0
    if args.command == 'remove':
        if stations.pop(args.name, None) is None:
            print(f'없는 지점: {args.name}')
            return 1
        save_stations(args.file, stations)
        _print_list(stations, args.file)
        return 0

    if args.command == 'record' and not valid_name(args.name):
        print('지점 이름은 영문/숫자/_/- 40자 이내')
        return 1
    targets = []
    if args.command == 'go':
        targets = [args.name]
    elif args.command == 'tour':
        targets = list(args.names)
    missing = [name for name in targets if name not in stations]
    if missing:
        print(f'없는 지점: {missing}. 먼저 record 하세요.')
        _print_list(stations, args.file)
        return 1

    # Ctrl+C 때 rclpy 가 컨텍스트를 먼저 닫으면 Nav2 에 취소를 보낼 수 없다 → 신호 처리를 끈다.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = StationClient(args.status)
    try:
        if args.command in ('where', 'record'):
            pose = node.current_pose()
            if pose is None:
                print('map -> base_link TF 없음 (AMCL 위치 지정/지도 실행 확인)')
                return 1
            print(f'지금 위치: x={pose[0]:.3f} y={pose[1]:.3f} yaw={math.degrees(pose[2]):.1f}deg')
            if args.command == 'record':
                stations[args.name] = {'x': pose[0], 'y': pose[1], 'yaw': pose[2]}
                path = save_stations(args.file, stations)
                print(f'저장: {args.name} -> {path}')
            return 0

        if args.command == 'go':
            return 0 if node.go(args.name, stations[args.name]) else 2

        rounds = 0
        while True:
            rounds += 1
            for index, name in enumerate(targets):
                ok = node.go(name, stations[name])
                if not ok and not args.keep_going:
                    return 2
                last = index == len(targets) - 1 and not args.loop
                if not last and args.dwell > 0.0:
                    print(f'[stations] {args.dwell:.0f} s 대기')
                    end = time.monotonic() + args.dwell
                    while time.monotonic() < end:
                        rclpy.spin_once(node, timeout_sec=0.1)
            if not args.loop:
                return 0
            print(f'[stations] {rounds}바퀴 완료, 반복')
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: Optional[List[str]] = None) -> None:
    sys.exit(run(parse_args(sys.argv[1:] if argv is None else argv)))


if __name__ == '__main__':
    main()
