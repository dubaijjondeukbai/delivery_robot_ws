import math

import pytest
from robot_bringup import stations


def test_yaw_quaternion_round_trip():
    for yaw in (0.0, 0.5, -1.2, math.pi - 1e-6, -math.pi + 1e-6):
        assert stations.quaternion_to_yaw(*stations.yaw_to_quaternion(yaw)) == pytest.approx(yaw)


def test_save_and_load(tmp_path):
    path = str(tmp_path / 'maps' / 'stations.yaml')
    assert stations.load_stations(path) == {}
    data = {'booth_a': {'x': 1.23456, 'y': -2.0, 'yaw': 0.5}, 'home': {'x': 0, 'y': 0, 'yaw': 0}}
    stations.save_stations(path, data)
    loaded = stations.load_stations(path)
    assert list(loaded) == ['booth_a', 'home']
    assert loaded['booth_a'] == {'x': 1.2346, 'y': -2.0, 'yaw': 0.5}


def test_valid_name():
    assert stations.valid_name('booth_a')
    assert stations.valid_name('A-1')
    assert not stations.valid_name('부스')
    assert not stations.valid_name('a b')
    assert not stations.valid_name('')


def test_status_for_safety_sequence():
    assert stations.status_for_safety('STOP:obstacle_detected', False, False) is None
    assert stations.status_for_safety('STOP:obstacle_detected', True, False) == 'OBSTACLE_STOP'
    assert stations.status_for_safety('STOP:obstacle_detected', True, True) is None
    assert stations.status_for_safety('STOP:vision_hazard', True, False) == 'OBSTACLE_STOP'
    assert stations.status_for_safety('READY', True, True) == 'NAVIGATING'
    assert stations.status_for_safety('READY', True, False) is None
    assert stations.status_for_safety('STOP:estop_active', True, False) is None


def test_parse_args_strips_ros_args():
    args = stations.parse_args(['tour', 'a', 'b', '--loop', '--ros-args', '-r', 'x:=y'])
    assert args.command == 'tour'
    assert args.names == ['a', 'b']
    assert args.loop
    assert args.status
    assert stations.parse_args(['--no-status', 'go', 'a']).status is False


def test_initial_pose_message_and_init_command():
    msg = stations.initial_pose_message({'x': 1.5, 'y': -0.5, 'yaw': math.pi / 2})
    assert msg.header.frame_id == 'map'
    assert msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0   # AMCL 이 최신 TF 사용
    assert msg.pose.pose.position.x == 1.5 and msg.pose.pose.position.y == -0.5
    q = msg.pose.pose.orientation
    assert stations.quaternion_to_yaw(q.x, q.y, q.z, q.w) == pytest.approx(math.pi / 2)
    assert msg.pose.covariance[0] == pytest.approx(0.0625)
    assert msg.pose.covariance[7] == pytest.approx(0.0625)
    assert msg.pose.covariance[35] == pytest.approx(math.radians(15.0) ** 2)
    assert stations.parse_args(['init', 'HOME']).name == 'HOME'
