"""
모니터 화면 문구 만들기 (delivery_display·delivery_console 이 쓴다. ROS·pygame 의존성 없음).

/delivery/state 스냅샷, /safety/state, 마지막으로 읽은 QR 을 받아 화면 한 장의 내용(Screen)을 만든다.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

LID_TEXT = {
    'CLOSED': '닫힘',
    'CLOSED_UNCONFIRMED': '닫힘 (센서 확인 없음)',
    'OPEN': '열림',
    'OPEN_UNCONFIRMED': '열림 (센서 확인 없음)',
    'MOVING': '움직이는 중',
    'UNKNOWN': '알 수 없음',
}

# /safety/state 의 STOP:<사유> → (띠 문구, 주행 중에만 띄울지, 색)
SAFETY_TEXT = {
    'estop_active': ('비상 정지 버튼이 눌려 있습니다', False, 'error'),
    'estop_state_timeout': ('E-stop 신호 끊김: STM32 연결 확인', False, 'error'),
    'waiting_for_estop_state': ('E-stop 신호 대기 중: STM32 연결 확인', False, 'warn'),
    'obstacle_detected': ('앞에 장애물이 있어 멈췄습니다: 길을 비켜 주세요', True, 'warn'),
    'obstacle_state_timeout': ('LiDAR 신호 끊김', False, 'error'),
    'waiting_for_obstacle_state': ('LiDAR 신호 대기 중', False, 'warn'),
    'vision_hazard': ('사람이 가까이 있어 멈췄습니다', True, 'warn'),
    'lid_open': ('뚜껑이 닫혀 있지 않아 주행을 막았습니다', True, 'warn'),
    'lid_state_timeout': ('뚜껑 상태 신호 끊김: delivery_demo 확인', False, 'error'),
    'waiting_for_lid_state': ('뚜껑 상태 신호 대기 중: delivery_demo 확인', False, 'warn'),
    'rear_obstacle_detected': ('뒤에 장애물이 있어 후진하지 않습니다', False, 'warn'),
    'rear_obstacle_state_timeout': ('후방 LiDAR 판단 끊김: 후진 안 함', False, 'error'),
    'waiting_for_rear_obstacle_state': ('후방 LiDAR 판단 대기 중', False, 'warn'),
}

MOVING_STATES = ('NAVIGATING', 'RETURNING')
CLOSED_LIDS = ('CLOSED', 'CLOSED_UNCONFIRMED')


@dataclass
class Screen:
    tone: str          # idle / move / wait / action / done / error / offline (배경색)
    title: str         # 큰 글씨
    guide: str         # 다음에 할 일
    note: str = ''     # 작은 글씨 보충
    lid: str = ''      # 아래 띠: 뚜껑 상태
    qr: str = ''       # 아래 띠: 마지막으로 읽은 QR
    banner: str = ''   # 위 띠: Safety 정지 사유
    banner_tone: str = ''


def _by_state(state: str, target: str, arrived: str, lid: str, event: str) -> Screen:
    if state == 'IDLE':
        if lid not in CLOSED_LIDS:
            return Screen('wait', '대기 중', '뚜껑을 닫아야 출발합니다: 닫기 QR(CLOSE)')
        return Screen('idle', '대기 중', '출발 QR을 보여 주세요 (예: START:A)')
    if state == 'NAVIGATING':
        return Screen('move', f'{target} 지점으로 이동 중', '길을 비켜 주세요')
    if state == 'STOPPING':
        return Screen('wait', f'{arrived} 지점 도착', '멈추는 중입니다. 잠시 기다려 주세요')
    if state == 'WAIT_OPEN':
        return Screen('wait', f'{arrived} 지점 도착', f'도착 QR을 보여 주세요 (OPEN:{arrived})')
    if state == 'OPENING':
        return Screen('action', '뚜껑 여는 중', '로봇에서 손을 떼 주세요')
    if state in ('OPEN', 'OPEN_UNCONFIRMED'):
        screen = Screen('done', '뚜껑 열림', '물건을 꺼낸 뒤 닫기 QR(CLOSE)을 보여 주세요')
        if state == 'OPEN_UNCONFIRMED':
            screen.note = '뚜껑 센서가 없어 열림을 확인하지 못했습니다. 눈으로 확인하세요'
        return screen
    if state == 'CLOSING':
        return Screen('action', '뚜껑 닫는 중', '손을 조심하세요')
    if state == 'RETURNING':
        return Screen('move', f'{target} 지점으로 돌아가는 중', '길을 비켜 주세요')
    if state == 'ERROR':
        recover = 'RESET' if lid in CLOSED_LIDS else 'CLOSE 로 뚜껑을 닫은 뒤 RESET'
        return Screen('error', '오류', event or '원인 불명', note=f'운영자: {recover}')
    return Screen('offline', state or '?', event)


def describe(snapshot: Optional[dict], snapshot_age: float, safety: str = '',
             safety_age: float = float('inf'), last_qr: str = '',
             qr_age: float = float('inf'), stale_sec: float = 3.0,
             qr_show_sec: float = 30.0) -> Screen:
    """
    화면 한 장의 내용.

    *_age 는 그 메시지를 받은 뒤 지난 초(받은 적 없으면 inf).
    """
    fresh = snapshot is not None and snapshot_age <= stale_sec
    if not fresh:
        screen = Screen('offline', '연결 대기',
                        '시연 노드(delivery_demo)가 실행 중인지 확인하세요')
        state = ''
    else:
        state = str(snapshot.get('state') or '')
        lid = str(snapshot.get('lid') or '')
        screen = _by_state(state, str(snapshot.get('target') or ''),
                           str(snapshot.get('arrived') or ''), lid,
                           str(snapshot.get('last_event') or ''))
        screen.lid = '뚜껑: ' + LID_TEXT.get(lid, lid or '-')

    if safety_age <= stale_sec:
        if safety.startswith('STOP:'):
            reason = safety.split(':', 1)[1]
            text, moving_only, tone = SAFETY_TEXT.get(reason, ('', False, ''))
            if text and (state in MOVING_STATES or not moving_only):
                screen.banner, screen.banner_tone = text, tone
    elif fresh:
        screen.banner, screen.banner_tone = 'Safety 신호 없음 (robot.launch.py 확인)', 'warn'

    if last_qr and qr_age <= qr_show_sec:
        screen.qr = f'읽은 QR: {last_qr} ({int(qr_age)}초 전)'
    return screen
