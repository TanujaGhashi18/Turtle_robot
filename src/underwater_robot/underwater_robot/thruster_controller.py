import rclpy
from rclpy.node import Node


class ThrusterController(Node):

    def __init__(self):
        super().__init__('thruster_controller')

        self.get_logger().info('Thruster controller started!')
        self.get_logger().info('Waiting for autonomous commands...')


def main(args=None):
    rclpy.init(args=args)

    node = ThrusterController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
