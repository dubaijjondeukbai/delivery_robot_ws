#!/usr/bin/env python3
"""
[Sim] 시뮬레이션 전용 E-stop 상태 발행 노드.

robot_safety/safety_manager_node 는 /estop_state (std_msgs/msg/Bool) 를
estop_timeout_sec 안에 주기적으로 받지 못하면 'waiting_for_estop_state' /
'estop_state_timeout' 으로 정지한다. 실물에서는 MCU(STM32/ESP32) 쪽
발행자가 이 토픽을 담당하지만 Gazebo 에는 그런 발행자가 없어서
Safety 가 영원히 STOP 에 머문다(빨간 경로는 그려지는데 로봇은 정지).
이 노드가 시뮬레이션에서 그 자리를 대신한다.

    /estop_state   std_msgs/msg/Bool   data=False(정상) 를 rate_hz 로 발행

실행 중 E-stop 동작을 시험하려면 파라미터를 바꾼다:
    ros2 param set /sim_estop_publisher estop_active true    # Safety -> STOP: estop_active
    ros2 param set /sim_estop_publisher estop_active false   # 해제(새 속도 명령부터 통과)

주의
----
* 시뮬레이션 전용이다. 실물 로봇에서는 절대 실행하지 않는다.
  robot_bringup/robot.launch.py 는 sim:=true 일 때만 이 노드를 띄우고,
  sim:=false 에서 sim_estop:=true 를 주면 launch 가 거부한다.
* 타이머는 Safety 와 같은 STEADY 시계를 쓴다. Gazebo 를 일시정지해도
  발행이 끊기지 않지만, 그때는 어차피 로봇이 움직이지 않는다.
"""

import math

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool


class SimEstopPublisher(Node):
    """시뮬레이션에서 /estop_state 하트비트를 대신 발행한다."""

    def __init__(self):
        super().__init__('sim_estop_publisher')

        self.declare_parameter('topic', '/estop_state')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('estop_active', False)

        topic = self.get_parameter('topic').value
        rate_hz = self.get_parameter('rate_hz').value
        estop_active = self.get_parameter('estop_active').value

        if not isinstance(topic, str) or not topic.strip():
            raise ValueError('topic must be a non-empty string')

        if (
            isinstance(rate_hz, bool)
            or not isinstance(rate_hz, (int, float))
            or not math.isfinite(float(rate_hz))
            or float(rate_hz) <= 0.0
        ):
            raise ValueError('rate_hz must be a finite number > 0')

        if not isinstance(estop_active, bool):
            raise ValueError('estop_active must be a bool')

        self._topic = topic.strip()
        self._rate_hz = float(rate_hz)
        self._estop_active = estop_active

        # Safety 입력(Best Effort)과 호환되는 Reliable 발행.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._publisher = self.create_publisher(Bool, self._topic, qos)

        # use_sim_time 과 무관하게 돌아가는 하트비트 (Safety 의 감시 시계와 동일).
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            1.0 / self._rate_hz,
            self._publish,
            clock=self._steady_clock,
        )

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().warning(
            '[SimEstop] SIMULATION ONLY | '
            f'publishing {self._topic} data={self._estop_active} '
            f'at {self._rate_hz:.1f} Hz'
        )

        self._publish()

    def _on_set_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name != 'estop_active':
                continue

            if parameter.type_ != Parameter.Type.BOOL:
                return SetParametersResult(
                    successful=False,
                    reason='estop_active must be a bool',
                )

            self._estop_active = bool(parameter.value)
            self.get_logger().warning(
                f'[SimEstop] estop_active -> {self._estop_active}'
            )

        return SetParametersResult(successful=True)

    def _publish(self):
        msg = Bool()
        msg.data = self._estop_active
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SimEstopPublisher()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
