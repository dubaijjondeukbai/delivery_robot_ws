"""
safety_manager_node 의 선택 후진 탈출 시험 (test_vision_hazard_input.py 와 같은 방식).

* allow_reverse_escape=false(기본): 후진 명령은 0 으로 잘린다(설정 파일의 max_reverse_velocity 와 무관).
* true: 전방 장애물이 있어도 후방이 비어 있으면 느린 직선 후진만 통과(회전 0),
  후방 장애물·후방 판단 끊김이면 정지, 전진 명령은 여전히 전방 장애물에 막힘.
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

    def _make(allow, max_reverse=0.08):
        rclpy.init(args=['--ros-args', '-p',
                         f'allow_reverse_escape:={str(allow).lower()}',
                         '-p', f'max_reverse_velocity:={max_reverse}',
                         '-p', 'rear_obstacle_timeout_sec:=0.5',
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


def _cmd(v, w=0.0):
    msg = Twist()
    msg.linear.x = v
    msg.angular.z = w
    return msg


def _last(node):
    msg = node.safe_cmd_pub.messages[-1]
    return msg.linear.x, msg.angular.z, node.state_pub.messages[-1].data


def test_default_never_reverses(make_node):
    node, _ = make_node(False, max_reverse=0.3)
    assert node.rear_sub is None and node.max_reverse_velocity == 0.0
    node.estop_callback(Bool(data=False))
    node.obstacle_callback(Bool(data=False))
    node.cmd_callback(_cmd(-0.2))
    assert _last(node)[0] == 0.0


def test_reverse_escape_past_front_obstacle_when_rear_clear(make_node):
    node, clock = make_node(True)
    node.estop_callback(Bool(data=False))
    node.obstacle_callback(Bool(data=True))             # 벽 앞 0.8 m 안
    node.rear_obstacle_callback(Bool(data=False))       # 뒤는 비어 있음
    node.cmd_callback(_cmd(0.2))
    assert _last(node) == (0.0, 0.0, 'STOP:obstacle_detected')   # 전진은 여전히 막힘

    node.cmd_callback(_cmd(-0.15, 0.3))                  # Nav2 BackUp
    v, w, state = _last(node)
    assert v == pytest.approx(-0.08) and w == 0.0 and state == 'READY'   # 0.08 로 제한, 직선만

    clock['t'] += 0.1
    node.obstacle_callback(Bool(data=True))             # 후진 중 전방 판단은 무시
    assert _last(node)[2] == 'READY'

    clock['t'] += 0.1
    before = len(node.safe_cmd_pub.messages)
    node.rear_obstacle_callback(Bool(data=True))        # 뒤에 사람이 들어옴 → 그 콜백에서 정지
    assert len(node.safe_cmd_pub.messages) == before + 1
    assert _last(node) == (0.0, 0.0, 'STOP:rear_obstacle_detected')
    clock['t'] += 0.1
    node.cmd_callback(_cmd(-0.08))
    assert _last(node) == (0.0, 0.0, 'STOP:rear_obstacle_detected')


def test_reverse_needs_fresh_rear_state(make_node):
    node, clock = make_node(True)
    node.estop_callback(Bool(data=False))
    node.obstacle_callback(Bool(data=False))
    node.cmd_callback(_cmd(-0.05))
    assert _last(node)[2] == 'STOP:waiting_for_rear_obstacle_state'
    node.rear_obstacle_callback(Bool(data=False))
    node.cmd_callback(_cmd(-0.05))
    assert _last(node)[2] == 'READY'
    clock['t'] += 0.6                                    # 후방 판단만 끊김
    node.estop_callback(Bool(data=False))
    node.obstacle_callback(Bool(data=False))
    node.cmd_callback(_cmd(-0.05))
    assert _last(node) == (0.0, 0.0, 'STOP:rear_obstacle_state_timeout')


def test_estop_still_stops_reverse(make_node):
    node, _ = make_node(True)
    node.estop_callback(Bool(data=False))
    node.obstacle_callback(Bool(data=True))
    node.rear_obstacle_callback(Bool(data=False))
    node.cmd_callback(_cmd(-0.08))
    assert _last(node)[2] == 'READY'
    node.estop_callback(Bool(data=True))
    assert _last(node) == (0.0, 0.0, 'STOP:estop_active')


def test_allow_without_reverse_speed_is_rejected():
    rclpy.init(args=['--ros-args', '-p', 'allow_reverse_escape:=true',
                     '-p', 'max_reverse_velocity:=0.0'])
    try:
        with pytest.raises(ValueError):
            SafetyManagerNode()
    finally:
        rclpy.try_shutdown()
