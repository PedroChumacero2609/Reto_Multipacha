import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math

class FigureDetector(Node):
    def __init__(self):
        super().__init__('figure_detector')  # Inicia el nodo
        
        # Crea una suscripción al tópico 'figure_cloud'
        self.subscriber = self.create_subscription(
            PointCloud2,           # Tipo de mensaje: PointCloud2
            'figure_cloud',        # Nombre del tópico
            self.callback,         # Función que se llama al recibir un mensaje
            10                     # Profundidad de la cola (buffer)
        )

    def callback(self, msg):
        # Convierte el mensaje PointCloud2 en una lista de puntos (x, y, z)
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        num_points = len(points)

        # Si hay muy pocos puntos, no intenta detectar figura
        if num_points < 10:
            return

        # Calcula la distancia de cada punto al origen (0, 0) en 2D (ignora z)
        distances = [math.sqrt(x**2 + y**2) for x, y, _ in points]

        # Promedio del radio (distancia al centro)
        avg_radius = sum(distances) / len(distances)

        # Desviación estándar del radio: mide qué tan uniforme es la distancia
        std_radius = math.sqrt(sum((r - avg_radius) ** 2 for r in distances) / len(distances))

        # Clasifica la figura en función de la desviación estándar:
        # - Si es muy baja → los puntos están a igual distancia → círculo
        # - Si es alta → más dispersos → triángulo
        # - SI es intermedio → cuadrado
        if std_radius < 0.02:
            figura = "círculo"
        elif std_radius >= 0.08:
            figura = "triángulo"
        else:
            figura = "cuadrado"

        # Muestra en consola qué figura fue detectada
        self.get_logger().info(
            f"Detectado: {figura.upper()}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = FigureDetector()
    rclpy.spin(node)      
    rclpy.shutdown() 