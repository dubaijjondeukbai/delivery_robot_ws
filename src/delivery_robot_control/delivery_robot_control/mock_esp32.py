import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MockESP32(Node):
    def __init__(self):
        super().__init__('mock_esp32')

        self.subscription = self.create_subscription(
            String, 'esp32_command', self.command_callback, 10
        )
        self.status_publisher = self.create_publisher(
            String, 'esp32_status', 10
        )

        self.operation_timer = None
        self.pending_command = None

        self.get_logger().info('Mock ESP32 ready')

    def command_callback(self, msg):
        command = msg.data
                # LED 명령은 다른 동작 중에도 바로 처리
        led_commands = [
            'FRONT_LED_ON',
            'FRONT_LED_OFF',
            'REAR_LED_ON',
            'REAR_LED_OFF',
        ]

        if command in led_commands:
            response = String()
            response.data = f'{command}_DONE'
            self.status_publisher.publish(response)
            self.get_logger().info(f'Sent: {response.data}')
            return

        # 리프트 정지는 일반 명령보다 먼저 처리
        if command == 'LIFT_STOP':
            if self.pending_command in ['LIFT_UP', 'LIFT_DOWN']:
                self.operation_timer.cancel()
                self.destroy_timer(self.operation_timer)
                self.operation_timer = None
                self.pending_command = None

                self.get_logger().info('Lift operation cancelled')

            response = String()
            response.data = 'LIFT_STOPPED'
            self.status_publisher.publish(response)
            self.get_logger().info('Sent: LIFT_STOPPED')
            return

        durations = {
            'LOCK': 1.0,
            'UNLOCK': 1.0,
            'LIFT_UP': 5.0,
            'LIFT_DOWN': 5.0,
        }

        if command not in durations:
            self.get_logger().warning(f'Unknown command: {command}')
            return

        if self.operation_timer is not None:
            response = String()
            response.data = f'BUSY:{command}'
            self.status_publisher.publish(response)
            self.get_logger().warning(f'Busy: rejected {command}')
            return

        self.pending_command = command
        self.get_logger().info(f'{command} received: processing...')

        self.operation_timer = self.create_timer(
            durations[command], self.finish_operation
        )

    def finish_operation(self):
        self.operation_timer.cancel()
        self.destroy_timer(self.operation_timer)
        self.operation_timer = None

        response = String()
        response.data = f'{self.pending_command}_DONE'
        self.pending_command = None

        self.status_publisher.publish(response)
        self.get_logger().info(f'Sent: {response.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MockESP32()

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