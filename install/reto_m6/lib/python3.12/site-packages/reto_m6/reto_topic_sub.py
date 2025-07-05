import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RetoSubscriber(Node):
    def __init__(self):
        super().__init__('reto_topic_sub')
        self.subscription = self.create_subscription(
            String,
            'reto_topic',
            self.listener_callback,
            10)
        self.subscription  # evitar warning por variable sin usar
        self.get_logger().info('Suscriptor iniciado, escuchando...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Recibido: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    nodo = RetoSubscriber()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()