from robot_bringup import delivery_logic as d


def machine(**kwargs):
    config = d.DeliveryConfig(stations=frozenset({'A', 'B', 'HOME'}), **kwargs)
    return d.DeliveryMachine(config, now=0.0)


def kinds(actions):
    return [a for a in actions if a[0] != 'event']


def deliver_to_wait_open(m, name='A'):
    assert kinds(m.on_text(f'START:{name}', 1.0)) == [('goal', name)]
    assert kinds(m.on_nav_result(True, '', 5.0)) == [('stm32', 'STOP')]
    assert m.state == d.STOPPING
    m.on_stop_confirmed(5.5)
    assert m.state == d.WAIT_OPEN


def test_parse_command():
    assert d.parse_command('START:A') == ('START', 'A')
    assert d.parse_command(' open : b ') == ('OPEN', 'b')
    assert d.parse_command('CLOSE') == ('CLOSE', '')
    assert d.parse_command('RESET') == ('RESET', '')
    assert d.parse_command('START:') is None
    assert d.parse_command('START:A B') is None
    assert d.parse_command('hello') is None


def test_full_happy_path_with_limit_switch():
    m = machine()
    deliver_to_wait_open(m)
    assert kinds(m.on_text('OPEN:A', 6.0)) == [('esp32', 'LID_OPEN')]
    assert m.state == d.OPENING
    m.on_lid_line('BUSY:LID_OPEN', 7.0)
    m.on_lid_line('LID_OPENED', 9.0)
    assert m.state == d.OPEN and m.lid == d.LID_OPEN
    assert kinds(m.on_text('CLOSE:A', 20.0)) == [('esp32', 'LID_CLOSE')]
    m.on_lid_line('LID_CLOSED', 23.0)
    assert m.state == d.IDLE and m.lid == d.LID_CLOSED


def test_start_not_resent_while_moving_and_open_ignored():
    m = machine()
    m.on_text('START:A', 1.0)
    assert kinds(m.on_text('START:A', 1.5)) == []
    assert kinds(m.on_text('START:B', 2.0)) == []
    assert kinds(m.on_text('OPEN:A', 3.0)) == []
    assert m.state == d.NAVIGATING and m.target == 'A'


def test_open_requires_stop_confirmation_and_matching_destination():
    m = machine()
    m.on_text('START:A', 1.0)
    m.on_nav_result(True, '', 5.0)
    assert kinds(m.on_text('OPEN:A', 5.1)) == []          # 아직 정지 확인 전
    m.on_stop_confirmed(5.5)
    assert kinds(m.on_text('OPEN:B', 6.0)) == []          # 목적지 불일치
    assert 'B' in m.last_event and m.state == d.WAIT_OPEN
    assert kinds(m.on_text('OPEN:A', 6.5)) == [('esp32', 'LID_OPEN')]


def test_unconfirmed_open_is_not_success():
    m = machine()
    deliver_to_wait_open(m)
    m.on_text('OPEN:A', 6.0)
    m.on_lid_line('LID_OPEN_STEPS_DONE', 9.0)
    assert m.state == d.OPEN_UNCONFIRMED
    assert m.lid == d.LID_OPEN_UNCONFIRMED
    relaxed = machine(require_confirmed_open=False)
    deliver_to_wait_open(relaxed)
    relaxed.on_text('OPEN:A', 6.0)
    relaxed.on_lid_line('LID_OPEN_STEPS_DONE', 9.0)
    assert relaxed.state == d.OPEN


def test_does_not_start_with_lid_open_or_unknown():
    m = machine(assume_closed_at_start=False)
    assert kinds(m.on_text('START:A', 1.0)) == []
    m.on_lid_line('LID_STATE:CLOSED', 1.5)
    assert kinds(m.on_text('START:A', 2.0)) == [('goal', 'A')]
    other = machine()
    other.on_lid_line('LID_STATE:OPEN', 0.5)
    assert kinds(other.on_text('START:A', 1.0)) == []


def test_unknown_station_and_nav_failure():
    m = machine()
    assert kinds(m.on_text('START:Z', 1.0)) == []
    m.on_text('START:B', 2.0)
    m.on_nav_result(False, 'ABORTED 105', 30.0)
    assert m.state == d.ERROR
    assert kinds(m.on_text('OPEN:B', 31.0)) == []
    m.on_text('RESET', 32.0)
    assert m.state == d.IDLE


def test_timeouts_stop_lid_and_never_open_without_stop():
    m = machine(stop_timeout_sec=2.0, lid_timeout_sec=5.0)
    m.on_text('START:A', 1.0)
    m.on_nav_result(True, '', 5.0)
    assert kinds(m.tick(8.0)) == []
    assert m.state == d.ERROR                              # 정지 확인 실패
    n = machine(lid_timeout_sec=5.0)
    deliver_to_wait_open(n)
    n.on_text('OPEN:A', 6.0)
    assert kinds(n.tick(12.0)) == [('esp32', 'LID_STOP')]
    assert n.state == d.ERROR and n.lid == d.LID_UNKNOWN
    assert kinds(n.on_text('CLOSE', 13.0)) == [('esp32', 'LID_CLOSE')]   # ERROR 에서 닫기


def test_lid_error_and_close_unconfirmed():
    m = machine()
    deliver_to_wait_open(m)
    m.on_text('OPEN:A', 6.0)
    assert kinds(m.on_lid_line('LID_ERROR:DRIVER_FAULT', 7.0)) == [('esp32', 'LID_STOP')]
    assert m.state == d.ERROR
    strict = machine(require_confirmed_close=True)
    deliver_to_wait_open(strict)
    strict.on_text('OPEN:A', 6.0)
    strict.on_lid_line('LID_OPENED', 8.0)
    strict.on_text('CLOSE', 9.0)
    strict.on_lid_line('LID_CLOSE_STEPS_DONE', 12.0)
    assert strict.state == d.ERROR


def test_auto_close_return_home_and_start_cooldown():
    m = machine(auto_close_sec=10.0, return_to='HOME', start_cooldown_sec=3.0)
    deliver_to_wait_open(m)
    m.on_text('OPEN:A', 6.0)
    m.on_lid_line('LID_OPENED', 8.0)
    assert kinds(m.tick(17.0)) == []
    assert kinds(m.tick(18.5)) == [('esp32', 'LID_CLOSE')]
    assert kinds(m.on_lid_line('LID_CLOSED', 21.0)) == [('goal', 'HOME')]
    assert m.state == d.RETURNING
    assert kinds(m.on_text('OPEN:A', 22.0)) == []
    m.on_nav_result(True, '', 40.0)
    assert m.state == d.IDLE
    assert kinds(m.on_text('START:A', 41.0)) == []         # 같은 QR 을 아직 들고 있음
    assert kinds(m.on_text('START:A', 44.0)) == [('goal', 'A')]


def test_recovery_close_after_reset_does_not_return_home():
    m = machine(lid_timeout_sec=5.0, return_to='HOME')
    deliver_to_wait_open(m)
    m.on_text('OPEN:A', 6.0)
    m.tick(12.0)                                           # 열기 시간 초과
    assert m.state == d.ERROR and m.lid == d.LID_UNKNOWN
    m.on_text('RESET', 13.0)
    assert m.state == d.IDLE and m.lid == d.LID_UNKNOWN    # 닫혔다고 가정하지 않음
    assert kinds(m.on_text('START:B', 14.0)) == []
    assert kinds(m.on_text('CLOSE', 15.0)) == [('esp32', 'LID_CLOSE')]   # IDLE 에서 복구용 닫기
    assert kinds(m.on_lid_line('LID_CLOSE_STEPS_DONE', 18.0)) == []      # HOME 으로 가지 않음
    assert m.state == d.IDLE and m.lid == d.LID_CLOSED_UNCONFIRMED
    assert kinds(m.on_text('CLOSE', 19.0)) == []           # 이미 닫힘: 무시
    assert kinds(m.on_text('START:B', 20.0)) == [('goal', 'B')]


def test_cancel_while_moving():
    m = machine()
    m.on_text('START:A', 1.0)
    assert kinds(m.on_text('CANCEL', 2.0)) == [('cancel',)]
    assert m.state == d.IDLE


def test_motion_after_arrival_stops_lid():
    m = machine()
    deliver_to_wait_open(m)
    m.on_text('OPEN:A', 6.0)
    assert kinds(m.on_unexpected_motion(7.0)) == [('esp32', 'LID_STOP')]
    assert m.state == d.ERROR
    idle = machine()
    assert kinds(idle.on_unexpected_motion(1.0)) == []
