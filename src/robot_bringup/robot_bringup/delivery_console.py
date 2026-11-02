"""
운영자 콘솔: QR 없이 시연 명령을 보내고 상태를 본다 (노트북, 또는 Pi 에 SSH 로 들어가서).

    ros2 run robot_bringup delivery_console

    a / b     START:A / START:B   출발
    oa / ob   OPEN:A / OPEN:B     도착 확인 → 뚜껑 열기
    c         CLOSE               뚜껑 닫기 (오류 뒤 복구용으로도)
    x         CANCEL              이동 취소 (그 자리에 정지)
    r         RESET               오류 해제
    s         지금 상태 다시 보기
    START:C 처럼 ':' 가 들어간 줄은 그대로 보낸다. q 또는 Ctrl+D 로 끝.

명령은 /delivery/command 로 간다(QR 과 같은 규칙: 이동 중 OPEN 무시, 목적지 다른 OPEN 거부 등).
"""

from __future__ import annotations

import json
import threading
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from robot_bringup.delivery_logic import parse_command
from robot_bringup.display_text import describe
from std_msgs.msg import String

SHORTCUTS = {
    'a': 'START:A', 'b': 'START:B',
    'oa': 'OPEN:A', 'ob': 'OPEN:B',
    'c': 'CLOSE', 'x': 'CANCEL', 'r': 'RESET',
}


def translate(line: str) -> Optional[str]:
    """입력 한 줄 → 보낼 명령. 모르는 입력은 None."""
    text = line.strip()
    if not text:
        return None
    command = SHORTCUTS.get(text.lower())
    if command is None:
        verb, colon, name = text.partition(':')
        command = f'{verb.strip().upper()}{colon}{name.strip()}'
    return command if parse_command(command) is not None else None


class Console(Node):

    def __init__(self) -> None:
        super().__init__('delivery_console')
        self.declare_parameter('command_topic', '/delivery/command')
        self.declare_parameter('state_topic', '/delivery/state')
        self.pub = self.create_publisher(
            String, self.get_parameter('command_topic').value, 10)
        self.create_subscription(
            String, self.get_parameter('state_topic').value, self._on_state, 10)
        self.snapshot: Optional[dict] = None
        self.snapshot_at = 0.0
        self.printed = None

    def _on_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(data, dict):
            return
        self.snapshot, self.snapshot_at = data, time.monotonic()
        key = (data.get('state'), data.get('target'), data.get('lid'), data.get('last_event'))
        if key != self.printed:
            self.printed = key
            self.show()

    def show(self) -> None:
        if self.snapshot is None:
            print('\n[상태] 아직 못 받음: Pi 의 delivery_demo 가 실행 중인지, 같은 네트워크·'
                  'ROS_DOMAIN_ID 인지 확인 (안 되면 Pi 에 SSH 로 들어가서 실행)', flush=True)
            return
        screen = describe(self.snapshot, time.monotonic() - self.snapshot_at)
        data = self.snapshot
        print(f"\n[상태] {data.get('state')} | {screen.title} | {screen.guide}"
              f"{' | ' + screen.note if screen.note else ''} | {screen.lid}\n"
              f"       마지막 이벤트: {data.get('last_event') or '-'}", flush=True)

    def send(self, command: str) -> None:
        self.pub.publish(String(data=command))
        print(f'[보냄] {command}', flush=True)


def _spin(executor: SingleThreadedExecutor) -> None:
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def main(args=None) -> None:
    # Ctrl+C 를 rclpy 가 가로채면 input() 이 안 끝난다 → 파이썬 기본 처리(KeyboardInterrupt)로 둔다.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Console()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=_spin, args=(executor,), daemon=True)
    spinner.start()
    print(__doc__.split('명령은')[0], flush=True)
    try:
        time.sleep(1.5)
        if node.snapshot is None:
            node.show()
        while rclpy.ok():
            try:
                line = input('명령> ')
            except EOFError:
                break
            if line.strip().lower() in ('q', 'quit', 'exit'):
                break
            if line.strip().lower() == 's':
                node.show()
                continue
            command = translate(line)
            if command is None:
                if line.strip():
                    print('모르는 명령입니다 (a, b, oa, ob, c, x, r, s, q 또는 START:A 형식)')
                continue
            node.send(command)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 스핀 스레드를 먼저 멈추고 기다린 뒤 정리한다(안 그러면 종료 때 abort 가 난다).
        executor.shutdown()
        spinner.join(timeout=3.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
