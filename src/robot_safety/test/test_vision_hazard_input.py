"""
safety_manager_node 의 선택 비전 위험 입력 시험.

DDS 지연 없이 판단 로직만 보도록 콜백을 직접 호출하고, 발행기를 기록용으로 바꾸며,
시계(_now)를 시험에서 정한다.

* use_vision_hazard=false(기본): /vision/person_hazard 를 무시하고 기존 동작 유지.
* true: True 를 받은 그 콜백에서 바로 0 을 발행(다음 타이머를 기다리지 않음),
  입력 단절 시 정지, 해제 후에는 새 명령이 와야 다시 움직임.
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

    def _make(use_vision):
        rclpy.init(args=['--ros-args', '-p',
                         f'use_vision_hazard:={str(use_vision).lower()}',
                         '-p', 'vision_hazard_timeout_sec:=0.5',
                         '-p', 'cmd_timeout_sec:=0.5'])
        node = SafetyManagerNode()
        node.control_timer.cancel()             # 시험에서는 출력 시점을 직접 정한다
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


def _healthy(node, vision=False):
    node.estop_callback(Bool(data=False))
    node.obstacle_callback(Bool(data=False))
    if node.use_vision_hazard and vision is not None:
        node.vision_hazard_callback(Bool(data=vision))


def _last(node):
    return node.safe_cmd_pub.messages[-1].linear.x, node.state_pub.messages[-1].data


def test_default_ignores_vision_topic(make_node):
    node, _ = make_node(False)
    assert node.vision_sub is None
    _healthy(node)
    node.cmd_callback(_cmd())
    assert _last(node) == (pytest.approx(0.2), 'READY')


def test_hazard_true_stops_in_same_callback(make_node):
    node, clock = make_node(True)
    _healthy(node, vision=False)
    node.cmd_callback(_cmd())
    assert _last(node) == (pytest.approx(0.2), 'READY')

    before = len(node.safe_cmd_pub.messages)
    node.vision_hazard_callback(Bool(data=True))
    # 콜백 하나 안에서 곧바로 정지 명령이 나간다(타이머 대기 없음)
    assert len(node.safe_cmd_pub.messages) == before + 1
    assert _last(node) == (0.0, 'STOP:vision_hazard')

    # 위험 중 명령은 저장되지 않는다
    clock['t'] += 0.1
    node.cmd_callback(_cmd())
    assert _last(node) == (0.0, 'STOP:vision_hazard')

    # 해제만으로는 움직이지 않고, 새 명령이 와야 통과
    clock['t'] += 0.1
    node.vision_hazard_callback(Bool(data=False))
    assert _last(node) == (0.0, 'STOP:waiting_for_new_command')
    node.cmd_callback(_cmd())
    assert _last(node) == (pytest.approx(0.2), 'READY')


def test_vision_input_timeout_stops(make_node):
    node, clock = make_node(True)
    _healthy(node, vision=False)
    node.cmd_callback(_cmd())
    assert _last(node)[1] == 'READY'
    clock['t'] += 0.6                      # 비전 입력만 0.6 s 끊김 (timeout 0.5)
    node.estop_callback(Bool(data=False))
    node.obstacle_callback(Bool(data=False))
    node.cmd_callback(_cmd())
    assert _last(node) == (0.0, 'STOP:vision_hazard_timeout')


def test_waiting_for_vision_state_at_start(make_node):
    node, _ = make_node(True)
    node.estop_callback(Bool(data=False))
    node.obstacle_callback(Bool(data=False))
    node.cmd_callback(_cmd())
    assert _last(node) == (0.0, 'STOP:waiting_for_vision_hazard')


def test_lidar_stop_still_wins_when_vision_clear(make_node):
    node, _ = make_node(True)
    _healthy(node, vision=False)
    node.obstacle_callback(Bool(data=True))
    node.cmd_callback(_cmd())
    assert _last(node) == (0.0, 'STOP:obstacle_detected')
