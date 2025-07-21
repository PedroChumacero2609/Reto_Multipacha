import rclpy # Librería principal de ROS2
from rclpy.node import Node # Clase base para crear nodos
from std_msgs.msg import String # Mensaje estándar para enviar texto por los tópicos


# Clase del publicador
class RetoPublisher(Node):
    def __init__(self):
        super().__init__('reto_topic_pub') # Nodo creado
        self.publisher_ = self.create_publisher(String, 'reto_topic', 10) # Tipo de mensaje: String
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz = 0.1 s
        self.get_logger().info('Publicador iniciado a 10 Hz') # Mensaje de inicio
    
    # Función de publicación
    def timer_callback(self):
        msg = String()
        msg.data = 'M6 Reto'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicando: "{msg.data}"') # Imprime cada 0.1 s: Publicando: M6 Reto

# Función principal
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