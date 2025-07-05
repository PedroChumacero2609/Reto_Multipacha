import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RetoPublisher(Node):
    def __init__(self):
        super().__init__('reto_topic_pub')
        self.publisher_ = self.create_publisher(String, 'reto_topic', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz = 0.1 s
        self.get_logger().info('Publicador iniciado a 10 Hz')

    def timer_callback(self):
        msg = String()
        msg.data = 'M6 Reto'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicando: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    nodo = RetoPublisher()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()