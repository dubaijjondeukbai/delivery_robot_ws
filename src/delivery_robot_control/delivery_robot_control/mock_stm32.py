import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MockSTM32(Node):
    def __init__(self):
        super().__init__('mock_stm32')

        self.subscription = self.create_subscription(
            String,
            'control_command',
            self.command_callback,
            10
        )

        self.status_publisher = self.create_publisher(
            String,
            'stm32_status',
            10
        )

        self.stop_timer = None
        self.get_logger().info('Mock STM32 ready')

    def command_callback(self, msg):
        if msg.data != 'STOP':
            self.get_logger().warning(
                f'Unknown command: {msg.data}'
            )
            return

        if self.stop_timer is not None:
            return

        self.get_logger().info('STOP received: stopping...')
        self.stop_timer = self.create_timer(
            2.0,
            self.finish_stop
        )

    def finish_stop(self):
        self.stop_timer.cancel()
        self.destroy_timer(self.stop_timer)
        self.stop_timer = None

        response = String()
        response.data = 'DRIVE_STOPPED'
        self.status_publisher.publish(response)

        self.get_logger().info('Sent: DRIVE_STOPPED')


def main(args=None):
    rclpy.init(args=args)
    node = MockSTM32()

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