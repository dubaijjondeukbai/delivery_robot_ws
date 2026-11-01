"""person_hazard 히스테리시스·주의 신호 시험 (ROS 불필요)."""

import math

from robot_vision.person_hazard import (
    ALERT,
    CLEAR,
    EgoMotion,
    HAZARD,
    HazardConfig,
    NONE,
    PersonHazardFilter,
    PersonObs,
    point_in_polygon,
    STALE,
)

DT = 0.2   # 5 Hz 검출
STILL = EgoMotion(v=0.0, w=0.0, known=True)


def near_center(track='a', score=0.8, dx=0.0):
    # 발 (0.5+dx, 0.95), 높이 0.6 -> 경로 영역 안 + 가까움
    return PersonObs(track, score, 0.42 + dx, 0.35, 0.58 + dx, 0.95)


def far_side(track='b', x=0.05):
    # 화면 왼쪽 끝 작은 사람 -> 영역 밖
    return PersonObs(track, 0.8, x, 0.50, x + 0.05, 0.62)


def make(**kw):
    cfg = HazardConfig(**kw)
    return PersonHazardFilter(cfg)


def feed(f, frames, start=0.0, ego=STILL):
    t = start
    states = []
    for people in frames:
        f.update(t, list(people), ego, now=t)
        states.append(f.state)
        t += DT
    return t, states


def test_zone_geometry():
    zone = HazardConfig().path_zone
    assert point_in_polygon(0.5, 0.95, zone)
    assert not point_in_polygon(0.05, 0.6, zone)


def test_entry_is_immediate_on_first_frame():
    f = make()
    f.update(0.0, [near_center()], STILL, now=0.0)
    assert f.state == HAZARD and f.hazard_active
    assert f.counters['hazard_entries'] == 1


def test_flicker_does_not_release():
    f = make(clear_frames=5, clear_sec=1.0)
    frames = [[near_center()] if i % 2 == 0 else [] for i in range(12)]
    _, states = feed(f, frames)
    assert all(s == HAZARD for s in states)
    assert f.counters['raw_toggles'] >= 10        # 원시 판정은 매 프레임 흔들렸지만
    assert f.counters['state_changes'] == 1       # 출력 상태는 한 번만 바뀜


def test_release_needs_frames_and_time_after_leaving():
    f = make(clear_frames=5, clear_sec=1.0, miss_grace_sec=0.5)
    t, _ = feed(f, [[near_center()]] * 3)
    # 같은 트랙이 화면 안에서 영역 밖으로 나간 것이 관측됨 -> 유예 없이 해제 카운트
    t, states = feed(f, [[far_side(track='a')]] * 4, start=t)
    assert states[-1] == HAZARD                   # 4 프레임(0.8 s)으로는 부족
    t, states = feed(f, [[far_side(track='a')]] * 3, start=t)
    assert states[-1] == CLEAR


def test_missing_detection_grace_delays_release():
    f = make(clear_frames=3, clear_sec=0.4, miss_grace_sec=0.7)
    t, _ = feed(f, [[near_center()]] * 2)
    # 검출이 사라짐(가림). 0.7 s 동안은 같은 자리에 있다고 본다.
    t, states = feed(f, [[]] * 3, start=t)
    assert all(s == HAZARD for s in states)
    assert 'grace' in f.reason
    t, states = feed(f, [[]] * 6, start=t)
    assert states[-1] == CLEAR


def test_new_hazard_during_release_is_immediate():
    f = make(clear_frames=5, clear_sec=1.0)
    t, _ = feed(f, [[near_center()]])
    t, _ = feed(f, [[far_side(track='a')]] * 3, start=t)
    assert 'releasing' in f.reason
    f.update(t, [near_center(track='z')], STILL, now=t)
    assert f.state == HAZARD and f.reason.startswith('enter')


def test_id_switch_keeps_history_and_logs():
    f = make()
    t, _ = feed(f, [[near_center(track='run:1:7')]] * 3)
    f.drain_events()
    # 추적기 재시작: 같은 사람에게 새 ID
    f.update(t, [near_center(track='run:2:1')], STILL, now=t)
    events = f.drain_events()
    assert any(e['event'] == 'ID_SWITCH' and e['old'] == 'run:1:7' for e in events)
    assert f.tracks['run:2:1'].observations == 4
    assert f.state == HAZARD


def test_id_switch_not_stolen_by_track_present_in_same_frame():
    f = make()
    t, _ = feed(f, [[near_center(track='a'), far_side(track='b')]] * 2)
    # 'b' 가 같은 프레임에 있으므로, 'a' 자리에 나타난 'c' 만 'a' 를 승계해야 한다.
    f.update(t, [near_center(track='c'), far_side(track='b')], STILL, now=t)
    assert 'b' in f.tracks and 'c' in f.tracks and 'a' not in f.tracks


def test_stale_input_is_failsafe_and_requires_release():
    f = make(input_timeout_sec=1.0, clear_frames=2, clear_sec=0.2)
    feed(f, [[]] * 3)
    assert f.state != HAZARD
    f.tick(now=5.0)
    assert f.state == STALE and f.hazard_active
    feed(f, [[]] * 1, start=5.0)
    assert f.state == STALE                       # 재개 직후엔 아직 해제 아님
    feed(f, [[]] * 3, start=5.2)
    assert f.state == CLEAR and not f.hazard_active


def test_stale_not_hazard_when_configured():
    f = make(stale_is_hazard=False)
    f.update(0.0, [], STILL, now=0.0)
    f.tick(now=10.0)
    assert f.state == STALE and not f.hazard_active


def test_out_of_order_frames_are_ignored():
    f = make()
    f.update(1.0, [near_center()], STILL, now=1.0)
    f.update(0.5, [], STILL, now=1.1)
    assert f.counters['out_of_order'] == 1
    assert f.counters['frames'] == 1


def _walker(track, x, score=0.8):
    # 화면 옆에서 경로 쪽으로 걷는 사람(높이 0.3, 아직 '가까움' 아님)
    return PersonObs(track, score, x, 0.32, x + 0.08, 0.62)


def test_lateral_dart_toward_path_raises_alert_only_when_ego_still():
    f = make(lateral_speed_alert=0.30)
    xs = [0.05 + 0.08 * i for i in range(6)]     # 0.4 폭/s 로 중앙 쪽 이동(중앙선 전까지)
    t = 0.0
    for x in xs:
        f.update(t, [_walker('p', x)], STILL, now=t)
        t += DT
    track = f.tracks['p']
    assert track.attention == ALERT
    cues = track.attention_info
    assert cues['lateral_speed_toward_path_w_per_s'] > 0.3
    assert 'expansion_rate_per_s' in cues
    assert not any('m_per_s' in key or 'ttc' in key.lower() for key in cues)
    # 이 신호는 위험 상태(Safety 입력)를 바꾸지 않는다
    assert f.state == CLEAR or f.state == STALE


def test_crossing_inside_path_counts_as_toward():
    f = make(lateral_speed_alert=0.30)
    t = 0.0
    for i in range(6):      # 경로 영역 안(발 y 0.9)에서 중앙선을 넘어 오른쪽으로 가로지름
        x = 0.25 + 0.08 * i
        f.update(t, [PersonObs('p', 0.8, x, 0.60, x + 0.08, 0.90)], STILL, now=t)
        t += DT
    assert f.state == CLEAR      # 높이 0.3: '가까움'이 아니므로 위험 상태는 아님
    assert f.tracks['p'].attention_info['lateral_speed_toward_path_w_per_s'] > 0.3


def test_robot_rotation_makes_lateral_cue_unreliable():
    f = make()
    turning = EgoMotion(v=0.0, w=0.6, known=True)
    t = 0.0
    for i in range(6):
        f.update(t, [_walker('p', 0.05 + 0.08 * i)], turning, now=t)
        t += DT
    track = f.tracks['p']
    assert track.attention == NONE
    assert 'ego_rotation' in track.attention_info['reasons']


def test_unknown_ego_motion_gives_no_alert():
    f = make()
    t = 0.0
    for i in range(6):
        f.update(t, [_walker('p', 0.05 + 0.08 * i)], EgoMotion(known=False), now=t)
        t += DT
    assert f.tracks['p'].attention == NONE
    assert 'ego_motion_unknown' in f.tracks['p'].attention_info['reasons']


def test_looming_expansion_only_when_robot_stationary():
    def grow(i):
        h = 0.20 * math.exp(0.7 * i * DT)          # d ln h / dt = 0.7 /s
        return PersonObs('q', 0.8, 0.70, 0.62 - h, 0.78, 0.62)
    f = make(expansion_rate_alert=0.5)
    t = 0.0
    for i in range(6):
        f.update(t, [grow(i)], STILL, now=t)
        t += DT
    assert f.tracks['q'].attention == ALERT
    f2 = make(expansion_rate_alert=0.5)
    t = 0.0
    for i in range(6):
        f2.update(t, [grow(i)], EgoMotion(v=0.3, w=0.0, known=True), now=t)
        t += DT
    assert f2.tracks['q'].attention == NONE
    assert 'ego_forward_motion' in f2.tracks['q'].attention_info['reasons']
