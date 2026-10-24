
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class STM32SerialBridge(Node):
    def __init__(self):
        super().__init__('stm32_serial_bridge')

        self.declare_parameter('port', '/tmp/ttySTM32')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.serial_conn = None

        try:
            self.serial_conn = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0
            )
            self.get_logger().info(f'Serial connected: {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial connection failed: {e}')

    

        self.create_subscription(
            String, 'control_command',
            self.command_callback, 10
        )

        self.status_pub = self.create_publisher(
            String, 'stm32_status', 10
        )

        self.bridge_pub = self.create_publisher(
            String, 'bridge_status', 10
        )

        self.rx_buffer = bytearray()
        self.create_timer(0.02, self.read_serial)

    def command_callback(self, msg):
        if self.serial_conn is None:
            self.publish_bridge_status('COMM_FAILED')
            return

        try:
            data = (msg.data + '\n').encode('utf-8')
            self.serial_conn.write(data)
            self.get_logger().info(f'TX: {msg.data}')
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'TX error: {e}')
            self.publish_bridge_status('COMM_FAILED')

    def read_serial(self):
        if self.serial_conn is None:
            return

        try:
            waiting = self.serial_conn.in_waiting
            if waiting == 0:
                return

            self.rx_buffer.extend(
                self.serial_conn.read(waiting)
            )

            while b'\n' in self.rx_buffer:
                line, _, remaining = self.rx_buffer.partition(b'\n')
                self.rx_buffer = bytearray(remaining)

                response = line.decode(
                    'utf-8', errors='replace'
                ).strip()

                if response:
                    msg = String()
                    msg.data = response
                    self.status_pub.publish(msg)
                    self.get_logger().info(f'RX: {response}')

        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'RX error: {e}')
            self.publish_bridge_status('COMM_FAILED')

    def publish_bridge_status(self, status):
        msg = String()
        msg.data = status
        self.bridge_pub.publish(msg)

    def destroy_node(self):
        if self.serial_conn is not None:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STM32SerialBridge()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
