import rclpy  # Librería principal de ROS2
from rclpy.node import Node  # Clase base para crear nodos
from std_msgs.msg import String  # Tipo de mensaje estándar para texto (usado también por el publicador)


# Clase del suscriptor
class RetoSubscriber(Node):
    def __init__(self):
        super().__init__('reto_topic_sub') # Nodo creado
        self.subscription = self.create_subscription(
            String,             # Tipo de mensaje: String
            'reto_topic',       # Nombre del tópico suscrito
            self.listener_callback,
            10)
        self.subscription  # evitar warning por variable sin usar
        self.get_logger().info('Suscriptor iniciado, escuchando...') # Mensaje de inicio
    
    # Función de recepción
    def listener_callback(self, msg):
        self.get_logger().info(f'Recibido: "{msg.data}"') # Imprime el mensaje recibido

#Función principal
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