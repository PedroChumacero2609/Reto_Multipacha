import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math

class CirclePointDetector(Node):
    def __init__(self):
        super().__init__('circle_point_detector')  # Inicializa el nodo
        # Se suscribe al tópico 'random_circle_cloud'
        self.subscriber = self.create_subscription(
            PointCloud2,
            'random_circle_cloud',
            self.callback,
            10
        )
        self.radius = 1.0      # Radio del círculo
        self.epsilon = 0.01    # Margen de tolerancia para excluir los puntos del borde exacto

    def callback(self, msg):
        # Lee los puntos (x, y, z) desde el mensaje PointCloud2
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        inside_count = 0  # Contador de puntos dentro del círculo (excluyendo el borde)
        total = 0         # Contador total de puntos considerados (también sin el borde)

        for x, y, _ in points:
            r = math.sqrt(x**2 + y**2)  # Distancia radial al origen

            # Ignora puntos que están justo en el borde del círculo (±epsilon)
            if abs(r - self.radius) < self.epsilon:
                continue

            # Cuenta los puntos que están dentro del círculo
            if r < self.radius:
                inside_count += 1

            total += 1  # Solo cuenta los puntos que no fueron descartados por estar en el borde

        # Muestra en la consola el total y cuántos están dentro
        self.get_logger().info(
            f'Puntos totales: {total}, Dentro del círculo (sin borde): {inside_count}'
        )

def main(args=None):
    rclpy.init(args=args)            
    node = CirclePointDetector()    
    rclpy.spin(node)            
    rclpy.shutdown()   