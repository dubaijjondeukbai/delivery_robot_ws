"""
단일 전방 카메라 구성이 URDF / Gazebo 센서 / bridge / vision.yaml / launch 에서 일치하는지 시험.

colcon test 환경(설치된 share 디렉터리)에서 실행된다. ROS 노드는 띄우지 않는다.
"""

import ast
import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
import yaml


def _share(package, *parts):
    return os.path.join(get_package_share_directory(package), *parts)


def _urdf(sim):
    path = _share('robot_description', 'urdf', 'robot.urdf.xacro')
    doc = xacro.process_file(path, mappings={'sim': 'true' if sim else 'false'})
    return ET.fromstring(doc.toxml())


def _params(path, node):
    with open(path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    return data.get(node, {}).get('ros__parameters', {}), data


@pytest.fixture(scope='module')
def vision():
    camera, data = _params(_share('robot_vision', 'config', 'vision.yaml'), 'camera_node')
    return camera, data


def test_vision_yaml_single_camera(vision):
    camera, data = vision
    assert 'camera_node_2' not in data, '2번 카메라는 주석 예시로만 남겨야 한다'
    assert camera['image_topic'] == '/camera/image_raw'
    assert camera['camera_info_topic'] == '/camera/camera_info'
    assert camera['frame_id'] == 'camera_optical_frame'
    for node in ('qr_node', 'detector_node', 'pose_node', 'apriltag_node'):
        assert data[node]['ros__parameters']['image_topic'] == camera['image_topic'], node
    assert data['apriltag_node']['ros__parameters']['camera_info_topic'] == \
        camera['camera_info_topic']


def test_robot_yaml_has_no_second_camera():
    _, data = _params(_share('robot_bringup', 'config', 'robot.yaml'), 'camera_node')
    assert 'camera_node_2' not in data


def test_urdf_has_one_camera_with_optical_frame(vision):
    camera, _ = vision
    for sim in (False, True):
        robot = _urdf(sim)
        links = {link.get('name') for link in robot.findall('link')}
        assert 'camera_link' in links and camera['frame_id'] in links
        assert not any(name.startswith('camera_2') for name in links)
        joints = {j.get('name'): j for j in robot.findall('joint')}
        optical = joints['camera_optical_joint']
        assert optical.find('parent').get('link') == 'camera_link'
        assert optical.find('child').get('link') == camera['frame_id']
        rpy = [float(v) for v in optical.find('origin').get('rpy').split()]
        assert rpy == pytest.approx([-math.pi / 2, 0.0, -math.pi / 2])
        mount = joints['camera_joint']
        assert mount.find('parent').get('link') == 'base_link'
        xyz = [float(v) for v in mount.find('origin').get('xyz').split()]
        assert xyz[1] == pytest.approx(0.0), '현재 구성은 전방 중앙(y=0) 카메라'
        assert xyz[0] > 0.0, '카메라는 base_link 앞쪽'


def test_gazebo_sensor_matches_vision(vision):
    camera, _ = vision
    robot = _urdf(True)
    sensors = [s for s in robot.iter('sensor') if s.get('type') == 'camera']
    assert len(sensors) == 1, 'Gazebo 카메라 센서는 1개여야 한다'
    sensor = sensors[0]
    parent = [g for g in robot.findall('gazebo') if sensor in list(g)][0]
    assert parent.get('reference') == 'camera_link'
    assert sensor.find('topic').text == camera['image_topic']
    assert sensor.find('gz_frame_id').text == camera['frame_id']
    assert sensor.find('camera/camera_info_topic').text == camera['camera_info_topic']
    width = int(sensor.find('camera/image/width').text)
    height = int(sensor.find('camera/image/height').text)
    assert (width, height) == (camera['image_width'], camera['image_height'])


def test_bridge_has_camera_topics_only_for_one_camera(vision):
    camera, _ = vision
    with open(_share('robot_sim', 'config', 'bridge.yaml'), encoding='utf-8') as handle:
        entries = yaml.safe_load(handle)
    by_topic = {e['ros_topic_name']: e for e in entries}
    image = by_topic[camera['image_topic']]
    info = by_topic[camera['camera_info_topic']]
    assert image['ros_type_name'] == 'sensor_msgs/msg/Image'
    assert info['ros_type_name'] == 'sensor_msgs/msg/CameraInfo'
    assert image['direction'] == info['direction'] == 'GZ_TO_ROS'
    assert not any(topic.startswith('/camera_2') for topic in by_topic)


def test_launch_does_not_start_second_camera():
    path = _share('robot_bringup', 'launch', 'robot.launch.py')
    with open(path, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    literals = {n.value for n in ast.walk(tree)
                if isinstance(n, ast.Constant) and isinstance(n.value, str)}
    assert 'camera_node_2' not in literals
    declared = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                and getattr(n.func, 'id', '') == 'DeclareLaunchArgument'
                and n.args and getattr(n.args[0], 'value', None) == 'enable_camera_2']
    assert not declared
