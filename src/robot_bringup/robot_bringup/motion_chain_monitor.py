#!/usr/bin/env python3
r"""
명령 흐름 관찰 노드 (구독만 한다. 어떤 토픽에도 명령을 발행하지 않는다).

    Nav2 -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_auto -> safety_manager
         -> /cmd_vel_safe -> Gazebo(또는 실물 bridge) -> /wheel/odom -> EKF -> /odom

용도
----
"경로는 보이는데 로봇이 안 움직인다"를 단계별로 나눠 기록한다.

* 1 초마다 각 단계의 명령/측정값과 /safety/state, E-stop/장애물 입력 나이, TF 를 한 줄로 출력
* 상태가 바뀌거나 단계가 끊길 때 이벤트 로그
* /rosout 에서 Nav2 실패 문자열을 분류:
    INVALID_PATH_0_POSES ("Resulting plan has 0 poses")    -> 경로 변환 실패(103)
    FAILED_TO_MAKE_PROGRESS ("Failed to make progress")    -> 경로는 있으나 진행 없음(105)
  둘은 원인이 다르므로 따로 센다.
* /initialpose(RViz 2D Pose Estimate) 수신과 map->odom 점프를 기록하고,
  그 뒤 initialpose_window_sec 안에 생긴 Nav2 실패를 연결해 보여 준다.
* 회전 명령 구간에서만 /cmd_vel_safe, /wheel/odom, /imu/data, /odom (+ 선택: 지면 실측)
  의 yaw 를 적분해 비교한다. 정지 중 0 끼리의 비교로 EKF 를 판정하지 않는다.
* 명령이 0 인데 바퀴/IMU 가 움직이면 'UNCOMMANDED_MOTION' 이벤트(급발진 감시 관찰용).

실행 예
-------
    ros2 run robot_bringup motion_chain_monitor --ros-args -p use_sim_time:=true \\
        -p output_file:=/tmp/chain.jsonl

종료(Ctrl+C) 시 요약을 출력한다. 실물 로봇에서도 안전하게 쓸 수 있다(관찰 전용).
"""

from __future__ import annotations

import json
import math
import time
from typing import Dict, List, Optional

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from robot_bringup.chain_diagnosis import (
    classify_rosout,
    diagnose,
    interpret_rotation,
    lidar_stop_reason,
    RotationComparator,
    safety_category,
    safety_reason,
    Snapshot,
    Thresholds,
    TwistSample,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, String
import tf2_ros

ACTIVE_GOAL = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
               GoalStatus.STATUS_CANCELING)
STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}
WATCHED_NODES = ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
                 'velocity_smoother', 'amcl', 'safety_manager_node', 'lidar_obstacle_node',
                 'ekf_filter_node')


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class MotionChainMonitor(Node):
    """구독만 하는 진단 노드."""

    def __init__(self):
        super().__init__('motion_chain_monitor')

        defaults = {
            'cmd_vel_nav_topic': '/cmd_vel_nav',
            'cmd_vel_dry_run_topic': '/cmd_vel_nav_dry_run',
            'cmd_vel_auto_topic': '/cmd_vel_auto',
            'cmd_vel_safe_topic': '/cmd_vel_safe',
            'safety_state_topic': '/safety/state',
            'estop_topic': '/estop_state',
            'obstacle_topic': '/safety/obstacle_detected',
            'front_distance_topic': '/safety/front_distance',
            'wheel_odom_topic': '/wheel/odom',
            'imu_topic': '/imu/data',
            'odom_topic': '/odom',
            'amcl_pose_topic': '/amcl_pose',
            'initialpose_topic': '/initialpose',
            'plan_topic': '/plan',
            'nav_status_topic': '/navigate_to_pose/_action/status',
            # 선택: 지면 실측(시뮬레이션 전용, nav_msgs/Odometry). 비워 두면 사용 안 함.
            # robot_sim 은 /sim/ground_truth/odom 으로 제공한다.
            'ground_truth_topic': '',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'report_period_sec': 1.0,
            # 시작 직후 아직 메시지를 못 받은 상태를 '끊김'으로 기록하지 않는 시간
            'warmup_sec': 5.0,
            'duration_sec': 0.0,
            'output_file': '',
            'map_odom_jump_m': 0.3,
            'map_odom_jump_rad': 0.2,
            'initialpose_window_sec': 5.0,
            'rotation_cmd_threshold': 0.1,
            'uncommanded_v': 0.05,
            'uncommanded_w': 0.15,
            'uncommanded_hold_sec': 0.5,
            'quiet': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in defaults}
        self.p = p

        self._t0 = time.monotonic()
        self._last: Dict[str, float] = {}
        self._twist: Dict[str, TwistSample] = {}
        self._imu_wz: Optional[float] = None
        self._safety_state: Optional[str] = None
        self._estop: Optional[bool] = None
        self._obstacle: Optional[bool] = None
        self._front_distance = float('nan')
        self._goal_status: Dict[bytes, int] = {}
        self._amcl_cov = None
        self._plan_len: Optional[int] = None
        self._truth_yaw: Optional[float] = None
        self._truth_t: Optional[float] = None
        self._truth_wz: Optional[float] = None
        self._map_odom: Optional[tuple] = None
        self._initialpose_times: List[float] = []
        self._uncmd_since: Optional[float] = None
        self._uncmd_active = False
        self._last_findings: List[str] = []
        self._state_time: Dict[str, float] = {}
        self._last_state_tick: Optional[float] = None
        self._events: List[dict] = []
        self._counts: Dict[str, int] = {}
        self._linked_failures: List[dict] = []
        self._lidar_reasons: Dict[str, int] = {}
        self._rotation = RotationComparator(w_on=float(p['rotation_cmd_threshold']))
        self._thresholds = Thresholds()
        self._out = open(p['output_file'], 'a', encoding='utf-8') if p['output_file'] else None

        best_effort = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                 reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        for key, topic in (('nav', p['cmd_vel_nav_topic']),
                           ('dry_run', p['cmd_vel_dry_run_topic']),
                           ('auto', p['cmd_vel_auto_topic']),
                           ('safe', p['cmd_vel_safe_topic'])):
            self.create_subscription(Twist, topic,
                                     lambda m, k=key: self._on_twist(k, m.linear.x, m.angular.z),
                                     best_effort)
        for key, topic in (('wheel', p['wheel_odom_topic']), ('odom', p['odom_topic'])):
            self.create_subscription(
                Odometry, topic,
                lambda m, k=key: self._on_twist(
                    k, m.twist.twist.linear.x, m.twist.twist.angular.z),
                best_effort)
        self.create_subscription(Imu, p['imu_topic'], self._on_imu, best_effort)
        self.create_subscription(String, p['safety_state_topic'], self._on_safety, best_effort)
        self.create_subscription(Bool, p['estop_topic'], self._on_estop, best_effort)
        self.create_subscription(Bool, p['obstacle_topic'], self._on_obstacle, best_effort)
        self.create_subscription(Float32, p['front_distance_topic'], self._on_front, best_effort)
        self.create_subscription(PoseWithCovarianceStamped, p['amcl_pose_topic'],
                                 self._on_amcl, best_effort)
        self.create_subscription(PoseWithCovarianceStamped, p['initialpose_topic'],
                                 self._on_initialpose, best_effort)
        self.create_subscription(Path, p['plan_topic'], self._on_plan, best_effort)
        self.create_subscription(GoalStatusArray, p['nav_status_topic'], self._on_status,
                                 best_effort)
        self.create_subscription(Log, '/rosout', self._on_rosout, latched)
        if p['ground_truth_topic']:
            self.create_subscription(Odometry, p['ground_truth_topic'], self._on_truth,
                                     best_effort)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._graph_warned: Dict[str, str] = {}
        self.create_timer(5.0, self._check_graph)
        self.create_timer(0.05, self._on_fast_tick)
        self.create_timer(float(p['report_period_sec']), self._on_report)
        self.get_logger().info(
            '[Monitor] observing only (no publishers). Ctrl+C prints a summary.')

    # ------------------------------------------------------------------
    def _now(self) -> float:
        return time.monotonic() - self._t0

    def _age(self, key: str) -> Optional[float]:
        t = self._last.get(key)
        return None if t is None else self._now() - t

    def _event(self, kind: str, **data):
        self._counts[kind] = self._counts.get(kind, 0) + 1
        record = {'t': round(self._now(), 3), 'event': kind, **data}
        self._events.append(record)
        if len(self._events) > 2000:
            self._events = self._events[-1000:]
        if self._out:
            self._out.write(json.dumps(record, ensure_ascii=False) + '\n')
            self._out.flush()
        if not self.p['quiet']:
            extra = ' '.join(f'{k}={v}' for k, v in data.items())
            self.get_logger().info(f'[Monitor] EVENT {kind} {extra}')

    # ------------------------------------------------------------------
    def _on_twist(self, key, v, w):
        self._last[key] = self._now()
        sample = self._twist.setdefault(key, TwistSample())
        sample.v, sample.w = float(v), float(w)

    def _on_imu(self, msg):
        self._last['imu'] = self._now()
        self._imu_wz = float(msg.angular_velocity.z)

    def _on_safety(self, msg):
        now = self._now()
        if msg.data != self._safety_state:
            self._event('SAFETY_STATE', state=msg.data, category=safety_category(msg.data))
        self._safety_state = msg.data
        self._last['safety'] = now

    def _on_estop(self, msg):
        if self._estop is None or bool(msg.data) != self._estop:
            self._event('ESTOP_VALUE', active=bool(msg.data))
        self._estop = bool(msg.data)
        self._last['estop'] = self._now()

    def _on_obstacle(self, msg):
        if self._obstacle is None or bool(msg.data) != self._obstacle:
            self._event('OBSTACLE_VALUE', detected=bool(msg.data),
                        front_distance=round(self._front_distance, 3))
        self._obstacle = bool(msg.data)
        self._last['obstacle'] = self._now()

    def _on_front(self, msg):
        self._front_distance = float(msg.data)

    def _on_amcl(self, msg):
        c = msg.pose.covariance
        self._amcl_cov = (c[0], c[7], c[35])
        self._last['amcl'] = self._now()

    def _on_initialpose(self, msg):
        now = self._now()
        self._initialpose_times.append(now)
        pose = msg.pose.pose
        self._event('INITIALPOSE', x=round(pose.position.x, 3), y=round(pose.position.y, 3),
                    yaw=round(_yaw(pose.orientation), 3), frame=msg.header.frame_id,
                    stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def _on_plan(self, msg):
        n = len(msg.poses)
        if self._plan_len is None or (n == 0) != (self._plan_len == 0):
            self._event('PLAN', poses=n)
        self._plan_len = n
        self._last['plan'] = self._now()

    def _on_status(self, msg):
        for status in msg.status_list:
            gid = bytes(status.goal_info.goal_id.uuid)
            old = self._goal_status.get(gid)
            if old != status.status:
                self._event('NAV_GOAL', status=STATUS_NAMES.get(status.status, status.status),
                            goal=gid.hex()[:8])
            self._goal_status[gid] = status.status

    def _on_rosout(self, msg):
        if not any(msg.name.endswith(n) for n in WATCHED_NODES):
            return
        if msg.name.endswith('lidar_obstacle_node'):
            reason = lidar_stop_reason(msg.msg)
            if reason is not None:
                # obstacle_in_stop_zone = 실제 장애물, 그 밖(scan_timeout 등) = 데이터 신선도/형식
                self._event('LIDAR_STOP', reason=reason,
                            front_distance=round(self._front_distance, 3))
                self._lidar_reasons[reason] = self._lidar_reasons.get(reason, 0) + 1
            return
        kind = classify_rosout(msg.msg)
        if kind is None:
            return
        now = self._now()
        self._event(kind, node=msg.name, level=int(msg.level), msg=msg.msg[:160])
        if kind in ('INVALID_PATH_0_POSES', 'FAILED_TO_MAKE_PROGRESS', 'GOAL_FAILED',
                    'RECOVERY_TIMEOUT'):
            window = float(self.p['initialpose_window_sec'])
            recent = [t for t in self._initialpose_times if 0.0 <= now - t <= window]
            if recent:
                self._linked_failures.append({'failure': kind,
                                              'sec_after_initialpose': round(now - recent[-1], 2)})

    def _on_truth(self, msg):
        # 각속도는 연속 자세의 yaw 차이로 계산한다(±pi 경계 처리). Gazebo OdometryPublisher 의
        # twist 는 yaw 가 ±pi 를 넘을 때 튀는 값이 나올 수 있어 쓰지 않는다.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        yaw = _yaw(msg.pose.pose.orientation)
        if self._truth_t is not None and stamp > self._truth_t:
            self._truth_wz = _wrap(yaw - self._truth_yaw) / (stamp - self._truth_t)
        if self._truth_t is None or stamp > self._truth_t:
            self._truth_yaw, self._truth_t = yaw, stamp
            self._last['truth'] = self._now()

    def _check_graph(self):
        """명령 토픽의 발행자 수와 메시지 타입을 확인한다(Twist/TwistStamped 혼용, 중복 발행)."""
        expected = {
            self.p['cmd_vel_nav_topic']: (1, 2),     # controller_server, behavior_server
            self.p['cmd_vel_auto_topic']: (1, 1),    # velocity_smoother 하나만
            self.p['cmd_vel_safe_topic']: (1, 1),    # safety_manager 하나만
        }
        for topic, (lo, hi) in expected.items():
            infos = self.get_publishers_info_by_topic(topic)
            types = sorted({i.topic_type for i in infos})
            names = sorted({i.node_name for i in infos})
            problem = ''
            if any(t != 'geometry_msgs/msg/Twist' for t in types):
                problem = f'type={types} (Safety/bridge 는 geometry_msgs/msg/Twist 만 받음)'
            elif infos and not lo <= len(infos) <= hi:
                problem = f'publishers={names} (여러 노드가 같은 명령 토픽에 발행)'
            if problem and self._graph_warned.get(topic) != problem:
                self._event('TOPIC_GRAPH', topic=topic, problem=problem)
            self._graph_warned[topic] = problem

    # ------------------------------------------------------------------
    def _sample(self, key: str) -> TwistSample:
        sample = self._twist.get(key, TwistSample())
        return TwistSample(sample.v, sample.w, self._age(key))

    def _lookup(self, parent: str, child: str):
        try:
            tf = self._tf_buffer.lookup_transform(parent, child, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        t = tf.transform.translation
        return t.x, t.y, _yaw(tf.transform.rotation)

    def _snapshot(self) -> Snapshot:
        s = Snapshot(
            nav=self._sample('nav'), dry_run=self._sample('dry_run'),
            auto=self._sample('auto'), safe=self._sample('safe'),
            wheel=self._sample('wheel'), odom=self._sample('odom'),
            imu_wz=self._imu_wz, imu_age=self._age('imu'),
            safety_state=self._safety_state, safety_age=self._age('safety'),
            estop=self._estop, estop_age=self._age('estop'),
            obstacle=self._obstacle, obstacle_age=self._age('obstacle'),
            goal_active=any(v in ACTIVE_GOAL for v in self._goal_status.values()),
        )
        map_odom = self._lookup(self.p['map_frame'], self.p['odom_frame'])
        odom_base = self._lookup(self.p['odom_frame'], self.p['base_frame'])
        s.tf_map_odom_ok = map_odom is not None
        s.tf_odom_base_ok = odom_base is not None
        if map_odom is not None:
            if self._map_odom is not None:
                dx = math.hypot(map_odom[0] - self._map_odom[0], map_odom[1] - self._map_odom[1])
                dyaw = abs(_wrap(map_odom[2] - self._map_odom[2]))
                if dx > self.p['map_odom_jump_m'] or dyaw > self.p['map_odom_jump_rad']:
                    self._event('MAP_ODOM_JUMP', d_xy=round(dx, 3), d_yaw=round(dyaw, 3))
            self._map_odom = map_odom
        return s

    def _is_fresh(self, key: str, max_age: float = 0.5) -> bool:
        age = self._age(key)
        return age is not None and age <= max_age

    def _fresh_rate(self, key: str, value: Optional[float], max_age: float = 0.5):
        return value if self._is_fresh(key, max_age) else None

    def _on_fast_tick(self):
        now = self._now()
        safe = self._twist.get('safe')
        rates = {
            'cmd': self._fresh_rate('safe', safe.w if safe else None),
            'wheel': self._fresh_rate('wheel', self._twist['wheel'].w
                                      if 'wheel' in self._twist else None),
            'imu': self._fresh_rate('imu', self._imu_wz),
            'odom': self._fresh_rate('odom', self._twist['odom'].w
                                     if 'odom' in self._twist else None),
            'truth': self._fresh_rate('truth', self._truth_wz),
        }
        # 각속도는 ROS 시간(시뮬레이션이면 /clock) 기준이므로 적분도 같은 시계로 한다.
        # (Gazebo 가 실시간보다 느리면 벽시계로 적분한 각도가 커진다.)
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        closed = self._rotation.update(ros_now, rates)
        if closed is not None:
            self._event('ROTATION_SEGMENT', duration=round(closed.duration, 2),
                        **{f'yaw_{k}': round(v, 3) for k, v in closed.yaw.items()})

        # 정지 사유별 누적 시간
        if self._last_state_tick is not None:
            reason = safety_reason(self._safety_state) or 'READY'
            self._state_time[reason] = self._state_time.get(reason, 0.0) + (
                now - self._last_state_tick)
        self._last_state_tick = now

        # 명령 0 인데 움직임: 급발진/미끄럼 관찰
        commanded = safe is not None and self._is_fresh('safe') and (
            abs(safe.v) > 1e-3 or abs(safe.w) > 1e-3)
        wheel = self._twist.get('wheel')
        moving = (
            (wheel is not None and self._is_fresh('wheel')
             and abs(wheel.v) > self.p['uncommanded_v'])
            or (self._is_fresh('imu') and self._imu_wz is not None
                and abs(self._imu_wz) > self.p['uncommanded_w']))
        if moving and not commanded:
            if self._uncmd_since is None:
                self._uncmd_since = now
            elif not self._uncmd_active and now - self._uncmd_since >= self.p[
                    'uncommanded_hold_sec']:
                self._uncmd_active = True
                self._event('UNCOMMANDED_MOTION',
                            wheel_v=round(wheel.v, 3) if wheel else None,
                            imu_wz=round(self._imu_wz or 0.0, 3))
        else:
            self._uncmd_since = None
            self._uncmd_active = False

    def _on_report(self):
        s = self._snapshot()
        findings = diagnose(s, self._thresholds)
        if self._now() < float(self.p['warmup_sec']):
            findings = []
        for finding in findings:
            if finding not in self._last_findings:
                self._event('FINDING', text=finding)
        self._last_findings = findings

        def fmt(sample: TwistSample) -> str:
            if sample.age is None:
                return '--'
            stale = '' if sample.age <= 0.5 else f'(old {sample.age:.1f}s)'
            return f'{sample.v:+.2f}/{sample.w:+.2f}{stale}'

        line = (f't={self._now():6.1f} safety={self._safety_state} '
                f'| nav={fmt(s.nav)} auto={fmt(s.auto)} safe={fmt(s.safe)} '
                f'| wheel={fmt(s.wheel)} imu_wz='
                f'{"--" if s.imu_wz is None else f"{s.imu_wz:+.2f}"} odom={fmt(s.odom)} '
                f'| estop={s.estop} obstacle={s.obstacle} front={self._front_distance:.2f} '
                f'| tf map->odom={s.tf_map_odom_ok} odom->base={s.tf_odom_base_ok} '
                f'| goal_active={s.goal_active} plan={self._plan_len}')
        if self._out:
            self._out.write(json.dumps({'t': round(self._now(), 3), 'report': line,
                                        'findings': findings}, ensure_ascii=False) + '\n')
            self._out.flush()
        if not self.p['quiet']:
            self.get_logger().info(line)

        duration = float(self.p['duration_sec'])
        if duration > 0.0 and self._now() >= duration:
            raise SystemExit(0)

    # ------------------------------------------------------------------
    def summary(self) -> dict:
        rotation = self._rotation.summary()
        result = {
            'elapsed_sec': round(self._now(), 1),
            'safety_time_sec': {k: round(v, 1) for k, v in sorted(
                self._state_time.items(), key=lambda kv: -kv[1])},
            'event_counts': dict(sorted(self._counts.items())),
            'nav2_failures': {k: self._counts.get(k, 0) for k in (
                'INVALID_PATH_0_POSES', 'FAILED_TO_MAKE_PROGRESS', 'RECOVERY_TIMEOUT',
                'GOAL_FAILED', 'GOAL_SUCCEEDED', 'BT_SERVER_TIMEOUT')},
            'lidar_stop_reasons': self._lidar_reasons,
            'failures_after_initialpose': self._linked_failures,
            'rotation': {k: (round(v, 3) if isinstance(v, float) else v)
                         for k, v in rotation.items()},
            'rotation_notes': interpret_rotation(rotation),
            'last_findings': self._last_findings,
        }
        return result

    def close(self):
        if self._out:
            self._out.write(json.dumps({'summary': self.summary()}, ensure_ascii=False) + '\n')
            self._out.close()
            self._out = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotionChainMonitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        if node is not None:
            summary = node.summary()
            print('\n===== motion_chain_monitor summary =====')
            print(json.dumps(summary, ensure_ascii=False, indent=2))
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
