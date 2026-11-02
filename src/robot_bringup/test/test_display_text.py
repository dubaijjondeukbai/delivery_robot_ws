from robot_bringup.delivery_console import translate
from robot_bringup.display_text import describe


def snap(state, lid='CLOSED_UNCONFIRMED', target=None, arrived=None, event=''):
    return {'state': state, 'lid': lid, 'target': target, 'arrived': arrived,
            'last_event': event}


def test_offline_when_no_or_stale_state():
    assert describe(None, float('inf')).tone == 'offline'
    assert describe(snap('IDLE'), 10.0).title == '연결 대기'


def test_mission_flow_texts():
    idle = describe(snap('IDLE'), 0.1, 'STOP:waiting_for_new_command', 0.1)
    assert idle.tone == 'idle' and 'START' in idle.guide and idle.banner == ''
    assert idle.lid == '뚜껑: 닫힘 (센서 확인 없음)'
    moving = describe(snap('NAVIGATING', target='A'), 0.1, 'READY', 0.1)
    assert moving.title == 'A 지점으로 이동 중' and moving.tone == 'move'
    wait = describe(snap('WAIT_OPEN', target='A', arrived='A'), 0.1, 'READY', 0.1)
    assert 'OPEN:A' in wait.guide
    opened = describe(snap('OPEN_UNCONFIRMED', lid='OPEN_UNCONFIRMED', arrived='A'), 0.1,
                      'READY', 0.1)
    assert opened.title == '뚜껑 열림' and '확인하지 못했습니다' in opened.note


def test_lid_not_closed_and_error_recovery_hint():
    blocked = describe(snap('IDLE', lid='UNKNOWN'), 0.1, 'READY', 0.1)
    assert 'CLOSE' in blocked.guide
    error = describe(snap('ERROR', lid='UNKNOWN', event='뚜껑 열기 시간 초과: 정지'), 0.1,
                     'READY', 0.1)
    assert error.tone == 'error' and error.guide.startswith('뚜껑 열기 시간 초과')
    assert 'CLOSE' in error.note and 'RESET' in error.note
    closed_error = describe(snap('ERROR', event='x'), 0.1, 'READY', 0.1)
    assert closed_error.note == '운영자: RESET'


def test_safety_banners():
    moving = snap('NAVIGATING', target='B')
    assert '장애물' in describe(moving, 0.1, 'STOP:obstacle_detected', 0.1).banner
    assert describe(snap('IDLE'), 0.1, 'STOP:obstacle_detected', 0.1).banner == ''
    estop = describe(snap('IDLE'), 0.1, 'STOP:estop_active', 0.1)
    assert estop.banner_tone == 'error' and '비상 정지' in estop.banner
    assert 'Safety' in describe(snap('IDLE'), 0.1).banner          # Safety 소식 없음


def test_last_qr_shown_briefly():
    assert describe(snap('IDLE'), 0.1, 'READY', 0.1, 'START:A', 4.2).qr == \
        '읽은 QR: START:A (4초 전)'
    assert describe(snap('IDLE'), 0.1, 'READY', 0.1, 'START:A', 99.0).qr == ''


def test_console_translate():
    assert translate('a') == 'START:A'
    assert translate('OB') == 'OPEN:B'
    assert translate(' c ') == 'CLOSE'
    assert translate('reset') == 'RESET'
    assert translate('start:C') == 'START:C'
    assert translate('hello') is None
    assert translate('') is None
