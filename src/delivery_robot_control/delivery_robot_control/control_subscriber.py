import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from delivery_robot_control.state_machine import DeliveryStateMachine



class ControlSubscriber(Node):

    def __init__(self):
        super().__init__('control_subscriber')

        self.state_machine = DeliveryStateMachine()

        self.stm32_subscription = self.create_subscription(
                    String,
                    'stm32_status',
                    self.stm32_status_callback,
                    10
                )
        
        self.esp32_subscription = self.create_subscription(
            String,
            'esp32_status',
            self.esp32_status_callback,
            10
        )
        
        

        self.subscription = self.create_subscription(
            String,
            'navigation_status',
            self.status_callback,
            10
        )

        self.bridge_subscription = self.create_subscription(
            String,
            'bridge_status',
            self.bridge_status_callback,
            10
        )

        self.reset_subscription = self.create_subscription(
            String,
            'control_reset',
            self.reset_callback,
            10
        )

        self.command_publisher = self.create_publisher(
            String,
            'control_command',
            10
        )

        self.esp32_command_publisher = self.create_publisher(
            String,
            'esp32_command',
            10
        )


        self.payload_subscription = self.create_subscription(
            String,
            'payload_command',
            self.payload_command_callback,
            10
        )

        self.drive_stopped = False
        self.stop_requested = False

        self.stop_requested_at = None
        self.stop_response_timeout = 3.0

        self.stop_watchdog_timer = self.create_timer(
            0.1,
            self.check_stop_timeout
        )

    def status_callback(self, msg):
        status = msg.data
        current_state = self.state_machine.state

        if status == 'NAVIGATING':
            if current_state == 'IDLE':
                event = 'START'
            elif current_state == 'OBSTACLE_STOP':
                event = 'RESUME'
            else:
                event = 'START'

        elif status == 'OBSTACLE_STOP':
            event = 'OBSTACLE_STOP'

        elif status == 'GOAL_REACHED':
            event = 'GOAL_REACHED'

        elif status == 'FAILED':
            event = 'FAILED'

        else:
            self.get_logger().warning(
                f'Unknown navigation status: {status}'
            )
            return

        success = self.state_machine.transition(event)

        if success:
            self.get_logger().info(
                f'State changed to: {self.state_machine.state}'
            )
            
            if self.state_machine.state == 'ARRIVED':
                self.drive_stopped = False
                self.stop_requested = True

                self.stop_requested_at = time.monotonic()

                command_msg = String()
                command_msg.data = 'STOP'
                self.command_publisher.publish(command_msg)

                self.get_logger().info(
                    'Control command sent: STOP'
                )

        else:
            self.get_logger().warning(
                f'Invalid transition: {self.state_machine.state} + {event}'
            )

    def bridge_status_callback(self, msg):
        status = msg.data

        if status == 'COMM_OK':
            self.get_logger().info('Bridge communication: OK')

        elif status in ['COMM_TIMEOUT', 'COMM_FAILED']:
            self.state_machine.transition('ESTOP')
            self.drive_stopped = False
            self.stop_requested = False
            self.stop_requested_at = None

            drive_stop = String()
            drive_stop.data = 'STOP'
            self.command_publisher.publish(drive_stop)

            lift_stop = String()
            lift_stop.data = 'LIFT_STOP'
            self.esp32_command_publisher.publish(lift_stop)

            self.get_logger().error(
                f'Bridge error detected: {status} -> ESTOP'
            )
            self.get_logger().info(
                'Stop commands sent: STOP + LIFT_STOP'
            )

        else:
            self.get_logger().warning(
                f'Unknown bridge status: {status}'
            )

    def reset_callback(self, msg):
        if msg.data != 'RESET':
            return

        success = self.state_machine.transition('RESET')
        
        if success:
            self.drive_stopped = False
            self.stop_requested = False
            self.stop_requested_at = None

            self.get_logger().info(
                'System reset: IDLE'
            )
            
        else:
            self.get_logger().warning(
                f'Reset not allowed from {self.state_machine.state}'
            )

    def payload_command_callback(self, msg):
        allowed_commands = [
            'LOCK',
            'UNLOCK',
            'LIFT_UP',
            'LIFT_DOWN',
            'LIFT_STOP',
            'FRONT_LED_ON',
            'FRONT_LED_OFF',
            'REAR_LED_ON',
            'REAR_LED_OFF',
        ]

        if msg.data not in allowed_commands:
            self.get_logger().warning(
                f'Unknown payload command: {msg.data}'
            )
            return
        
        restricted_commands = ['LIFT_UP', 'LIFT_DOWN', 'UNLOCK']

        if msg.data in restricted_commands:
            if self.state_machine.state != 'ARRIVED':
                self.get_logger().warning(
                    f'명령 차단: {msg.data} '
                    f'(현재 상태: {self.state_machine.state}, 도착 상태 필요)'
                )
                return

            if not self.drive_stopped:
                self.get_logger().warning(
                    f'명령 차단: {msg.data} (주행 정지 확인 필요)'
                )
                return
        command = String()
        command.data = msg.data
        self.esp32_command_publisher.publish(command)

        self.get_logger().info(
            f'ESP32 command sent: {command.data}'
        )

    def check_stop_timeout(self):
        if not self.stop_requested:
            return

        if self.stop_requested_at is None:
            return

        elapsed = time.monotonic() - self.stop_requested_at

        if elapsed < self.stop_response_timeout:
            return

        self.stop_requested = False
        self.stop_requested_at = None
        self.drive_stopped = False

        self.state_machine.transition('ESTOP')

        drive_stop = String()
        drive_stop.data = 'STOP'
        self.command_publisher.publish(drive_stop)

        lift_stop = String()
        lift_stop.data = 'LIFT_STOP'
        self.esp32_command_publisher.publish(lift_stop)

        self.get_logger().error(
            'STM32 정지 응답 시간 초과 -> ESTOP'
        )
        self.get_logger().info(
            'Stop commands sent: STOP + LIFT_STOP'
        )

    
    def stm32_status_callback(self, msg):
        if msg.data == 'DRIVE_STOPPED':
            if (
                self.state_machine.state == 'ARRIVED'
                and self.stop_requested
            ):
                self.drive_stopped = True
                self.stop_requested = False
                self.stop_requested_at = None
                self.get_logger().info('STM32 주행 정지 확인')
            else:
                self.get_logger().warning(
                    '현재 도착 정지 요청에 해당하지 않는 응답'
                )

        elif msg.data == 'DRIVE_MOVING':
            self.drive_stopped = False
            
            lift_stop = String()
            lift_stop.data = 'LIFT_STOP'
            self.esp32_command_publisher.publish(lift_stop)

            self.get_logger().warning(
                'STM32 주행 감지: 정지 확인 해제 + LIFT_STOP 전송'
            )

        else:
            self.get_logger().warning(
                f'Unknown STM32 status: {msg.data}'
            )

    def esp32_status_callback(self, msg):

        messages = {
            'LOCK_DONE': '적재함 잠금 완료',
            'UNLOCK_DONE': '적재함 잠금 해제 완료',
            'LIFT_UP_DONE': '리프트 상승 완료',
            'LIFT_DOWN_DONE': '리프트 하강 완료',
            'LIFT_STOPPED': '리프트 정지 응답 수신',
            'FRONT_LED_ON_DONE': '앞 LED 켜짐',
            'FRONT_LED_OFF_DONE': '앞 LED 꺼짐',
            'REAR_LED_ON_DONE': '뒤 LED 켜짐',
            'REAR_LED_OFF_DONE': '뒤 LED 꺼짐',
        }

        if msg.data in messages:
            self.get_logger().info(messages[msg.data])

        elif msg.data.startswith('BUSY:'):
            self.get_logger().warning(
                f'ESP32 동작 중: 명령 거절됨 ({msg.data})'
            )

        else:
            self.get_logger().warning(
                f'Unknown ESP32 status: {msg.data}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ControlSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()