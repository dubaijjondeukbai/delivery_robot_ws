import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from delivery_robot_bridge.mock_stm32 import MockSTM32


class CommandSubscriber(Node):

    def __init__(self):
        super().__init__('command_subscriber')

        self.stm32 = MockSTM32()

        self.subscription = self.create_subscription(
            String,
            'control_command',
            self.command_callback,
            10
        )

        self.status_publisher = self.create_publisher(
            String,
            'bridge_status',
            10
        )

    def command_callback(self, msg):
        command = msg.data

        self.get_logger().info(
            f'Bridge received command: {command}'
        )

        response = self.stm32.send_command(command)

        if response is None:
            self.get_logger().error(
                f'Communication timeout: {command}'
            )

            if response is None:
                self.get_logger().error(
                    f'Communication timeout: {command}'
                )

                status_msg = String()
                status_msg.data = 'COMM_TIMEOUT'
                self.status_publisher.publish(status_msg)

                return

        expected_response = {
            'STOP': 'ACK,STOP',
            'LIFT_UP': 'STATE,LIFT_TOP',
            'LIFT_DOWN': 'STATE,LIFT_HOME',
            'LOCK': 'STATE,LOCKED',
            'UNLOCK': 'STATE,UNLOCKED',
            'ESTOP': 'STATE,EMERGENCY_STOP'
        }

        if command not in expected_response:
            self.get_logger().error(
                f'Unknown command: {command}'
            )
            return

        if response == expected_response[command]:
            self.get_logger().info(
                f'Command success: {command}'
            )

            status_msg = String()
            status_msg.data = 'COMM_OK'
            self.status_publisher.publish(status_msg)
        else:
            self.get_logger().error(
                f'Command failed: {command}, response={response}'
            )

            status_msg = String()
            status_msg.data = 'COMM_FAILED'
            self.status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)

    node = CommandSubscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()