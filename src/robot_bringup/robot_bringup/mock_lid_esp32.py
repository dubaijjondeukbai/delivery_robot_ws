"""
가짜 ESP32 뚜껑 제어기 (ROS 토픽 수준). 펌웨어 없이 delivery_demo 흐름을 시험한다.

    ros2 run robot_bringup mock_lid_esp32                          # 리미트 스위치 없음(실물과 같음)
    ros2 run robot_bringup mock_lid_esp32 --ros-args -p limit_switches:=true
    ros2 run robot_bringup mock_lid_esp32 --ros-args -p fault:=error   # 1 초 뒤 LID_ERROR
    ros2 run robot_bringup mock_lid_esp32 --ros-args -p fault:=silent  # 응답 없음(시간 초과 시험)

/esp32_command 의 LID_OPEN / LID_CLOSE / LID_STOP / LID_STATUS 에 docs/esp32_lid_protocol.md
형식으로 답한다(firmware/esp32_lid 참고 펌웨어와 같은 동작). 처음에는 닫혀 있다.
실물 ESP32 브리지(delivery_robot_bridge esp32_serial_bridge)와 함께 띄우지 않는다.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class MockLid(Node):

    def __init__(self) -> None:
        super().__init__('mock_lid_esp32')
        # 2026-11-02 사용자: 실물에 리미트 스위치가 없는 것 같음 → 기본 false
        for name, default in (('move_sec', 3.0), ('limit_switches', False), ('fault', ''),
                              ('state_rate_hz', 2.0), ('command_topic', '/esp32_command'),
                              ('status_topic', '/esp32_status')):
            self.declare_parameter(name, default)
        self.move_sec = self.get_parameter('move_sec').value
        self.limits = self.get_parameter('limit_switches').value
        self.fault = self.get_parameter('fault').value
        self.pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)
        self.create_subscription(
            String, self.get_parameter('command_topic').value, self._on_command, 10)
        self.state = 'CLOSED'
        self.moving: str = ''
        self.done_at = 0.0
        self.create_timer(0.05, self._tick)
        if self.limits:
            self.create_timer(1.0 / self.get_parameter('state_rate_hz').value, self._state)
        self.get_logger().info(
            f'[MockLid] limit_switches={self.limits} move_sec={self.move_sec} '
            f'fault={self.fault or "-"}')

    def _say(self, text: str) -> None:
        self.pub.publish(String(data=text))
        self.get_logger().info(f'[MockLid] -> {text}')

    def _on_command(self, msg: String) -> None:
        command = msg.data.strip().upper()
        if command in ('LID_OPEN', 'LID_CLOSE'):
            if self.moving:
                self._say(f'BUSY:{self.moving}')
                return
            already = 'OPEN' if command == 'LID_OPEN' else 'CLOSED'
            if self.state == already and self.fault == '':  # 펌웨어처럼 이미 그 위치면 바로 응답
                self.moving, self.done_at = command, 0.0
                return
            self.moving = command
            self.state = 'MOVING'
            delay = 1.0 if self.fault == 'error' else self.move_sec
            self.done_at = time.monotonic() + delay
        elif command == 'LID_STOP':
            if self.moving:
                self.state = 'UNKNOWN'
            self.moving = ''
            self._say('LID_STOPPED')
        elif command == 'LID_STATUS':
            self._state()

    def _tick(self) -> None:
        if not self.moving or time.monotonic() < self.done_at:
            return
        command, self.moving = self.moving, ''
        if self.fault == 'silent':
            self.state = 'UNKNOWN'
            return
        if self.fault == 'error':
            self.state = 'UNKNOWN'
            self._say('LID_ERROR:DRIVER_FAULT')
            return
        opening = command == 'LID_OPEN'
        self.state = 'OPEN' if opening else 'CLOSED'
        if self.limits:
            self._say('LID_OPENED' if opening else 'LID_CLOSED')
        else:
            self._say('LID_OPEN_STEPS_DONE' if opening else 'LID_CLOSE_STEPS_DONE')

    def _state(self) -> None:
        # 스위치가 없으면 센서로 확인한 상태가 없으므로 UNKNOWN (펌웨어와 같음)
        state = self.state if self.limits else ('MOVING' if self.moving else 'UNKNOWN')
        self.pub.publish(String(data=f'LID_STATE:{state}'))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MockLid()
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
