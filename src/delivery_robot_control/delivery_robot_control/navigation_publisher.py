import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NavigationPublisher(Node):

    def __init__(self):
        super().__init__('navigation_publisher')

        self.publisher_ = self.create_publisher(
            String,
            'navigation_status',
            10
        )

        self.status_list = [
            'NAVIGATING',
            'OBSTACLE_STOP',
            'NAVIGATING',
            'GOAL_REACHED'
        ]

        self.index = 0

        self.timer = self.create_timer(
            2.0,
            self.publish_status
        )

    def publish_status(self):
        msg = String()
        msg.data = self.status_list[self.index]

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Publish: {msg.data}'
        )

        self.index += 1
        
        if self.index >= len(self.status_list):
            self.timer.cancel()
            self.get_logger().info('Normal delivery test finished')


def main(args=None):
    rclpy.init(args=args)

    node = NavigationPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()