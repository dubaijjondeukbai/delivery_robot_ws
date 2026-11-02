"""
safety_manager_node 의 선택 뚜껑 인터록 시험 (test_vision_hazard_input.py 와 같은 방식).

* use_lid_interlock=false(기본): /delivery/lid_closed 를 구독하지 않고 기존 동작 유지.
* true: 닫힘(True)을 받기 전·열림(False)·입력 끊김이면 정지, 다시 닫히면 새 명령이 와야 움직임.
"""

from geometry_msgs.msg import Twist
import pytest
import rclpy
from robot_safety.safety_manager_node import SafetyManagerNode
from std_msgs.msg import Bool


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def make_node():
    nodes = []

    def _make(use_lid):
        rclpy.init(args=['--ros-args', '-p',
                         f'use_lid_interlock:={str(use_lid).lower()}',
                         '-p', 'lid_state_timeout_sec:=1.0',
                         '-p', 'cmd_timeout_sec:=0.5'])
        node = SafetyManagerNode()
        node.control_timer.cancel()
        clock = {'t': 100.0}
        node._now = lambda: clock['t']
        node.safe_cmd_pub = Recorder()
        node.state_pub = Recorder()
        nodes.append(node)
        return node, clock

    yield _make
    for node in nodes:
        node.destroy_node()
    rclpy.try_shutdown()


def _cmd(v=0.2):
    msg = Twist()
    msg.linear.x = v
    return msg


def _healthy(node):
    node.estop_callback(Bool(data=False))
    node.obstacle_callback(Bool(data=False))


def _last(node):
    return node.safe_cmd_pub.messages[-1].linear.x, node.state_pub.messages[-1].data


def test_default_does_not_subscribe(make_node):
    node, _ = make_node(False)
    assert node.lid_sub is None
    _healthy(node)
    node.cmd_callback(_cmd())
    assert _last(node) == (pytest.approx(0.2), 'READY')


def test_waits_for_lid_state_then_blocks_when_open(make_node):
    node, clock = make_node(True)
    _healthy(node)
    node.cmd_callback(_cmd())
    assert _last(node) == (0.0, 'STOP:waiting_for_lid_state')

    node.lid_closed_callback(Bool(data=True))
    node.cmd_callback(_cmd())
    assert _last(node) == (pytest.approx(0.2), 'READY')

    before = len(node.safe_cmd_pub.messages)
    node.lid_closed_callback(Bool(data=False))           # 뚜껑 열림 → 그 콜백에서 바로 정지
    assert len(node.safe_cmd_pub.messages) == before + 1
    assert _last(node) == (0.0, 'STOP:lid_open')
    clock['t'] += 0.1
    node.cmd_callback(_cmd())                             # 열린 동안의 명령(teleop 등)은 막힘
    assert _last(node) == (0.0, 'STOP:lid_open')

    clock['t'] += 0.1
    node.lid_closed_callback(Bool(data=True))             # 닫혀도 새 명령이 와야 움직임
    assert _last(node) == (0.0, 'STOP:waiting_for_new_command')
    node.cmd_callback(_cmd())
    assert _last(node) == (pytest.approx(0.2), 'READY')


def test_lid_state_timeout_stops(make_node):
    node, clock = make_node(True)
    _healthy(node)
    node.lid_closed_callback(Bool(data=True))
    node.cmd_callback(_cmd())
    assert _last(node)[1] == 'READY'
    clock['t'] += 1.2                                     # delivery_demo 가 멈춤 (timeout 1.0)
    _healthy(node)
    node.cmd_callback(_cmd())
    assert _last(node) == (0.0, 'STOP:lid_state_timeout')


def test_obstacle_still_wins_when_lid_closed(make_node):
    node, _ = make_node(True)
    _healthy(node)
    node.lid_closed_callback(Bool(data=True))
    node.obstacle_callback(Bool(data=True))
    node.cmd_callback(_cmd())
    assert _last(node) == (0.0, 'STOP:obstacle_detected')
