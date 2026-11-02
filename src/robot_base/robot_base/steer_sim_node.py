"""
시뮬레이션 전용 조향 제한: Gazebo 의 스키드(DiffDrive) 로봇이 실물 조향처럼 움직이게 한다.

    /cmd_vel_safe -> [steer_sim_node] -> /cmd_vel_sim_steered -> Gazebo DiffDrive

실물(base_bridge_node)과 같은 계산(robot_base/swerve.py)과 같은 설정(config/base.yaml 의 /** 공통값)
으로 조향 방식(앞 2바퀴)·조향 범위·조향 속도·조향 대기(gate)를 흉내 내고, 바퀴가 실제로 만들 수 있는 몸체 속도
(정기구학 결과)만 Gazebo 로 보낸다. 그래서 조향 범위가 제자리 회전 각보다 작으면
시뮬레이션에서도 제자리 회전이 안 되고, 너무 급한 회전은 실물처럼 완만해진다.
robot.launch.py sim:=true 에서 sim_steer_limit:=true(기본)일 때 실행된다.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from robot_base import swerve


class SteerSim(Node):

    def __init__(self) -> None:
        super().__init__('steer_sim_node')
        defaults = {
            'wheel_x': 0.230,
            'wheel_y': 0.2706,
            'wheel_radius': 0.0857,
            'steer_mode': 'front',
            'max_steer_deg': 30.0,
            'max_wheel_rpm': 150.0,
            'steer_rate_dps': 180.0,
            'steer_gate_deg': 20.0,
            'rate_hz': 50.0,
            'cmd_timeout_sec': 0.3,
            'input_topic': '/cmd_vel_safe',
            'output_topic': '/cmd_vel_sim_steered',
        }
        self.p = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            self.p[name] = self.get_parameter(name).value

        self.geometry = swerve.SwerveGeometry(
            self.p['wheel_x'], self.p['wheel_y'], self.p['wheel_radius'],
            steer_mode=self.p['steer_mode'])
        self.limits = swerve.SwerveLimits(
            max_steer=math.radians(self.p['max_steer_deg']),
            max_wheel_speed=self.geometry.rpm_to_speed(self.p['max_wheel_rpm']),
        )
        self.gate = math.radians(self.p['steer_gate_deg'])
        self.angles = [0.0] * 4
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = None
        self.last_tick = None
        self.adjusted = 0

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Twist, self.p['output_topic'], qos)
        self.create_subscription(Twist, self.p['input_topic'], self._on_cmd, qos)
        self.create_timer(1.0 / self.p['rate_hz'], self._tick)

        need = math.degrees(self.geometry.in_place_steer_angle())
        band = swerve.infeasible_turn_band(self.geometry, self.limits.max_steer)
        radius = swerve.minimum_turning_radius(self.geometry, self.limits.max_steer)
        self.get_logger().info(
            f"[SteerSim] steer_mode={self.p['steer_mode']} "
            f"max_steer={self.p['max_steer_deg']:.1f}deg "
            f"(제자리 회전 필요 {need:.1f}deg: {'가능' if self.p['max_steer_deg'] >= need else '불가'}) "
            f'최소 회전반경(차체 중심) {radius:.2f} m '
            f"steer_rate={self.p['steer_rate_dps']:.0f}deg/s "
            f"{self.p['input_topic']} -> {self.p['output_topic']}")
        if band is not None:
            self.get_logger().info(
                f'[SteerSim] 실행할 수 없는 회전반경 {band[0]:.2f}~{band[1]:.2f} m')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: Twist) -> None:
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if all(math.isfinite(v) for v in values):
            self.cmd = values
            self.cmd_time = self._now()

    def _tick(self) -> None:
        now = self._now()
        dt = 0.0 if self.last_tick is None else max(0.0, min(0.2, now - self.last_tick))
        self.last_tick = now
        fresh = self.cmd_time is not None and now - self.cmd_time <= self.p['cmd_timeout_sec']
        twist = self.cmd if fresh else (0.0, 0.0, 0.0)

        result = swerve.inverse(self.geometry, self.limits, *twist, self.angles)
        targets = [wheel.angle for wheel in result.wheels]
        if result.adjusted:
            self.adjusted += 1
            self.get_logger().info(
                f'[SteerSim] 명령 조정({result.adjusted}): '
                f'({twist[0]:.2f},{twist[2]:.2f}) -> ({result.used[0]:.2f},{result.used[2]:.2f})',
                throttle_duration_sec=2.0)

        # 조향 서보: 속도 제한으로 목표를 따라간다
        step = math.radians(self.p['steer_rate_dps']) * dt
        for index, target in enumerate(targets):
            error = swerve.wrap_angle(target - self.angles[index])
            self.angles[index] += max(-step, min(step, error))

        # 실물 브리지와 같은 조향 대기: 조향이 덜 돌았으면 바퀴를 굴리지 않는다
        gate = swerve.steering_gate(targets, self.angles, self.gate)
        speeds = [wheel.speed * gate for wheel in result.wheels]
        vx, _, wz, _ = swerve.forward(self.geometry, self.angles, speeds)

        out = Twist()
        out.linear.x = vx
        out.angular.z = wz
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SteerSim()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
