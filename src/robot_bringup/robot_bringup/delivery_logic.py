"""
QR 배달 시연 상태기계. ROS 의존성 없음, 단위 시험 대상(delivery_demo 노드가 실행한다).

    IDLE --START:A--> NAVIGATING --Nav2 성공--> STOPPING --정지 확인--> WAIT_OPEN
    WAIT_OPEN --OPEN:A(도착지와 같음)--> OPENING --LID_OPENED--> OPEN
                                               --LID_OPEN_STEPS_DONE--> OPEN_UNCONFIRMED
    OPEN / OPEN_UNCONFIRMED --CLOSE--> CLOSING --LID_CLOSED / LID_CLOSE_STEPS_DONE--> IDLE
                                                (return_to 가 있으면 RETURNING 을 거쳐 IDLE)
    어느 단계든 실패·시간 초과 --> ERROR --RESET--> IDLE

규칙
* 이동 중(NAVIGATING/RETURNING)에는 OPEN 을 무시하고, START 를 다시 보여줘도 목표를 또 보내지 않는다.
* 도착지와 다른 OPEN:X 는 열지 않는다. 정지가 확인되기 전(STOPPING)에도 열지 않는다.
* 뚜껑이 닫혀 있지 않으면(OPEN·MOVING·UNKNOWN) 출발하지 않는다.
* 열림은 리미트 스위치로 확인된 LID_OPENED 만 성공이다. 스텝 수만 채운 LID_OPEN_STEPS_DONE 은
  OPEN_UNCONFIRMED 로 따로 보고한다(require_confirmed_open=False 면 성공으로 본다).
* 복구: ERROR 에서, 또는 IDLE 인데 뚜껑이 닫혀 있지 않을 때 CLOSE 로 뚜껑만 닫는다(복귀 이동 없음).
  RESET 은 ERROR 만 풀고 뚜껑 상태는 그대로 둔다(닫혔다고 가정하지 않는다).

함수는 할 일(Action) 목록을 돌려준다. 시간은 호출하는 쪽이 넘긴다(now, 초).
"""

from __future__ import annotations

from dataclasses import dataclass
import re
from typing import Iterable, List, Optional, Tuple

NAME = re.compile(r'^[A-Za-z0-9_\-]{1,40}$')

IDLE = 'IDLE'
NAVIGATING = 'NAVIGATING'
STOPPING = 'STOPPING'
WAIT_OPEN = 'WAIT_OPEN'
OPENING = 'OPENING'
OPEN = 'OPEN'
OPEN_UNCONFIRMED = 'OPEN_UNCONFIRMED'
CLOSING = 'CLOSING'
RETURNING = 'RETURNING'
ERROR = 'ERROR'

LID_CLOSED = 'CLOSED'                    # 리미트 스위치로 확인
LID_CLOSED_UNCONFIRMED = 'CLOSED_UNCONFIRMED'
LID_OPEN = 'OPEN'
LID_OPEN_UNCONFIRMED = 'OPEN_UNCONFIRMED'
LID_MOVING = 'MOVING'
LID_UNKNOWN = 'UNKNOWN'

Action = Tuple[str, ...]  # ('goal', name) ('cancel',) ('stm32', text) ('esp32', text)
#                           ('event', level, message)


@dataclass
class DeliveryConfig:
    stations: frozenset = frozenset()
    require_confirmed_open: bool = True
    require_confirmed_close: bool = False
    assume_closed_at_start: bool = True
    stop_timeout_sec: float = 5.0
    lid_timeout_sec: float = 30.0
    auto_close_sec: float = 0.0
    return_to: str = ''
    start_cooldown_sec: float = 3.0


def parse_command(text: str) -> Optional[Tuple[str, str]]:
    """'START:A' -> ('START', 'A'). 알 수 없는 형식은 None."""
    if not text:
        return None
    verb, _, target = text.strip().partition(':')
    verb, target = verb.strip().upper(), target.strip()
    if verb in ('START', 'OPEN'):
        return (verb, target) if NAME.match(target) else None
    if verb == 'CLOSE':
        return (verb, target) if not target or NAME.match(target) else None
    if verb in ('CANCEL', 'RESET') and not target:
        return verb, ''
    return None


class DeliveryMachine:

    def __init__(self, config: DeliveryConfig, now: float = 0.0) -> None:
        self.config = config
        self.state = IDLE
        self.target: Optional[str] = None    # 가는 중인 지점
        self.arrived: Optional[str] = None   # 도착해서 정지한 지점
        self.lid = LID_CLOSED_UNCONFIRMED if config.assume_closed_at_start else LID_UNKNOWN
        self.deadline: Optional[float] = None
        self.auto_close_at: Optional[float] = None
        self.idle_since = now - 1e9          # 시작 직후에는 쿨다운 없음
        self.last_start: Optional[str] = None
        self.return_after_close = False      # 배달을 마치고 닫을 때만 return_to 로 복귀
        self.last_event = ''

    # ------------------------------------------------------------------
    def snapshot(self) -> dict:
        return {
            'state': self.state,
            'target': self.target,
            'arrived': self.arrived,
            'lid': self.lid,
            'last_event': self.last_event,
        }

    def _event(self, actions: List[Action], level: str, message: str) -> None:
        self.last_event = message
        actions.append(('event', level, message))

    def _go(self, state: str, now: float, timeout: Optional[float] = None) -> None:
        self.state = state
        self.deadline = None if timeout is None else now + timeout
        if state == IDLE:
            self.idle_since = now

    def _lid_closed(self) -> bool:
        return self.lid in (LID_CLOSED, LID_CLOSED_UNCONFIRMED)

    # ------------------------------------------------------------------
    def on_text(self, text: str, now: float) -> List[Action]:
        """QR 문자열 또는 운영자 명령(/delivery/command)."""
        actions: List[Action] = []
        parsed = parse_command(text)
        if parsed is None:
            self._event(actions, 'debug', f'모르는 QR/명령 무시: {text!r}')
            return actions
        verb, name = parsed

        if verb == 'START':
            self._on_start(name, text, now, actions)
        elif verb == 'OPEN':
            self._on_open(name, now, actions)
        elif verb == 'CLOSE':
            self._on_close(name, now, actions)
        elif verb == 'CANCEL':
            if self.state in (NAVIGATING, RETURNING):
                actions.append(('cancel',))
                self.target = None
                self._go(IDLE, now)
                self._event(actions, 'warn', '이동 취소: 그 자리에 정지')
        elif verb == 'RESET':
            if self.state == ERROR:
                self.target = None
                self._go(IDLE, now)
                hint = '' if self._lid_closed() else ': 출발하려면 CLOSE 로 먼저 닫기'
                self._event(actions, 'info', f'RESET: IDLE (뚜껑 {self.lid}){hint}')
        return actions

    def _on_start(self, name: str, text: str, now: float, actions: List[Action]) -> None:
        if self.state != IDLE:
            self._event(actions, 'debug', f'{self.state} 중이라 {text} 무시')
            return
        if text == self.last_start and now - self.idle_since < self.config.start_cooldown_sec:
            self._event(actions, 'debug', f'방금 끝난 {text} 반복 무시')
            return
        if name not in self.config.stations:
            self._event(actions, 'warn', f'등록되지 않은 지점: {name} (stations record {name})')
            return
        if not self._lid_closed():
            self._event(actions, 'warn',
                        f'뚜껑이 닫혀 있지 않아 출발하지 않음 (lid={self.lid}): CLOSE 로 닫은 뒤 다시')
            return
        self.target, self.arrived, self.last_start = name, None, text
        self._go(NAVIGATING, now)
        actions.append(('goal', name))
        self._event(actions, 'info', f'{name} 로 출발')

    def _on_open(self, name: str, now: float, actions: List[Action]) -> None:
        if self.state != WAIT_OPEN:
            self._event(actions, 'info', f'{self.state} 중이라 OPEN:{name} 무시')
            return
        if name != self.arrived:
            self._event(actions, 'warn', f'목적지 불일치: 도착 {self.arrived}, QR {name} → 열지 않음')
            return
        self.lid = LID_MOVING
        self._go(OPENING, now, self.config.lid_timeout_sec)
        actions.append(('esp32', 'LID_OPEN'))
        self._event(actions, 'info', f'{name} 도착 확인: 뚜껑 열기')

    def _on_close(self, name: str, now: float, actions: List[Action]) -> None:
        delivering = self.state in (OPEN, OPEN_UNCONFIRMED)
        recovering = self.state == ERROR or (self.state == IDLE and not self._lid_closed())
        if not delivering and not recovering:
            self._event(actions, 'info', f'{self.state} 중이라 CLOSE 무시 (lid={self.lid})')
            return
        if delivering and name and name != self.arrived:
            self._event(actions, 'warn', f'목적지 불일치: 도착 {self.arrived}, CLOSE:{name}')
            return
        self.return_after_close = delivering
        self.lid = LID_MOVING
        self.auto_close_at = None
        self._go(CLOSING, now, self.config.lid_timeout_sec)
        actions.append(('esp32', 'LID_CLOSE'))
        self._event(actions, 'info', '뚜껑 닫기' if delivering else '뚜껑 닫기(복구)')

    # ------------------------------------------------------------------
    def on_nav_result(self, succeeded: bool, detail: str, now: float) -> List[Action]:
        actions: List[Action] = []
        if self.state == NAVIGATING:
            if succeeded:
                self.arrived = self.target
                self._go(STOPPING, now, self.config.stop_timeout_sec)
                actions.append(('stm32', 'STOP'))
                self._event(actions, 'info', f'{self.target} 도착: 정지 요청, 정지 확인 대기')
            else:
                self._go(ERROR, now)
                self._event(actions, 'error', f'{self.target} 이동 실패: {detail}')
        elif self.state == RETURNING:
            self.target = None
            if succeeded:
                self._go(IDLE, now)
                self._event(actions, 'info', '복귀 완료')
            else:
                self._go(ERROR, now)
                self._event(actions, 'error', f'복귀 실패: {detail}')
        return actions

    def on_stop_confirmed(self, now: float) -> List[Action]:
        actions: List[Action] = []
        if self.state == STOPPING:
            self._go(WAIT_OPEN, now)
            self._event(actions, 'info', f'정지 확인: OPEN:{self.arrived} QR 대기')
        return actions

    def on_lid_line(self, line: str, now: float) -> List[Action]:
        """ESP32 가 보낸 줄(/esp32_status)."""
        actions: List[Action] = []
        text = line.strip()
        upper = text.upper()

        if upper.startswith('LID_STATE:'):
            value = upper.split(':', 1)[1]
            known = {'OPEN': LID_OPEN, 'CLOSED': LID_CLOSED, 'MOVING': LID_MOVING}
            self.lid = known.get(value, LID_UNKNOWN)
            return actions
        if upper.startswith('BUSY:LID'):
            return actions

        if self.state == OPENING:
            if upper == 'LID_OPENED':
                self.lid = LID_OPEN
                self._finish_open(OPEN, '뚜껑 열림 확인(리미트 스위치)', now, actions)
            elif upper == 'LID_OPEN_STEPS_DONE':
                self.lid = LID_OPEN_UNCONFIRMED
                if self.config.require_confirmed_open:
                    self._finish_open(OPEN_UNCONFIRMED,
                                      '열림 동작은 끝났지만 확인 안 됨(센서 없음): 눈으로 확인',
                                      now, actions)
                else:
                    self._finish_open(OPEN, '뚜껑 열림(스텝 수 기준, 센서 확인 없음)', now, actions)
            elif upper.startswith('LID_ERROR'):
                self._lid_error(text, now, actions)
        elif self.state == CLOSING:
            if upper == 'LID_CLOSED':
                self.lid = LID_CLOSED
                self._finish_close('뚜껑 닫힘 확인(리미트 스위치)', now, actions)
            elif upper == 'LID_CLOSE_STEPS_DONE':
                self.lid = LID_CLOSED_UNCONFIRMED
                if self.config.require_confirmed_close:
                    self._go(ERROR, now)
                    self._event(actions, 'error', '닫힘 확인 안 됨(센서 없음): 확인 후 RESET')
                else:
                    self._finish_close('뚜껑 닫힘(스텝 수 기준, 확인 없음): 눈으로 확인', now, actions)
            elif upper.startswith('LID_ERROR'):
                self._lid_error(text, now, actions)
        return actions

    def _finish_open(self, state: str, message: str, now: float, actions: List[Action]) -> None:
        self._go(state, now)
        level = 'info' if state == OPEN else 'warn'
        self._event(actions, level, message)
        if self.config.auto_close_sec > 0.0:
            self.auto_close_at = now + self.config.auto_close_sec

    def _finish_close(self, message: str, now: float, actions: List[Action]) -> None:
        self._event(actions, 'info', message)
        returning, self.return_after_close = self.return_after_close, False
        if returning and self.config.return_to and self.config.return_to in self.config.stations:
            self.target = self.config.return_to
            self._go(RETURNING, now)
            actions.append(('goal', self.config.return_to))
            self._event(actions, 'info', f'{self.config.return_to} 로 복귀')
        else:
            self.target = None
            self._go(IDLE, now)

    def _lid_error(self, text: str, now: float, actions: List[Action]) -> None:
        self.lid = LID_UNKNOWN
        self._go(ERROR, now)
        actions.append(('esp32', 'LID_STOP'))
        self._event(actions, 'error', f'뚜껑 오류: {text}')

    def on_unexpected_motion(self, now: float) -> List[Action]:
        """도착 후(정지 확인 이후) 로봇이 움직이면: 뚜껑 모터를 멈추고 ERROR."""
        actions: List[Action] = []
        if self.state in (WAIT_OPEN, OPENING, OPEN, OPEN_UNCONFIRMED, CLOSING):
            if self.state in (OPENING, CLOSING):
                actions.append(('esp32', 'LID_STOP'))
                self.lid = LID_UNKNOWN
            self._go(ERROR, now)
            self._event(actions, 'error', '도착 후 로봇이 움직임: 뚜껑 동작 중지')
        return actions

    # ------------------------------------------------------------------
    def tick(self, now: float) -> List[Action]:
        """시간 초과와 자동 닫기."""
        actions: List[Action] = []
        if self.deadline is not None and now > self.deadline:
            if self.state == STOPPING:
                self._go(ERROR, now)
                self._event(actions, 'error', '정지 확인 시간 초과: 뚜껑을 열지 않음')
            elif self.state in (OPENING, CLOSING):
                what = '열기' if self.state == OPENING else '닫기'
                self.lid = LID_UNKNOWN
                self._go(ERROR, now)
                actions.append(('esp32', 'LID_STOP'))
                self._event(actions, 'error', f'뚜껑 {what} 시간 초과: 정지')
        if (self.auto_close_at is not None and now >= self.auto_close_at
                and self.state in (OPEN, OPEN_UNCONFIRMED)):
            self.auto_close_at = None
            actions.extend(self.on_text('CLOSE', now))
        return actions


def stations_from(names: Iterable[str]) -> frozenset:
    return frozenset(n for n in names if NAME.match(n))
