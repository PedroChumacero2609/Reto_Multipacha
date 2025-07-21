import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import math

class FigurePublisher(Node):
    def __init__(self):
        super().__init__('figure_publisher')  # Inicia el nodo
        self.publisher = self.create_publisher(PointCloud2, 'figure_cloud', 10)  # Publicador de PointCloud2
        self.timer = self.create_timer(5.0, self.publish_figure)  # Ejecuta cada 5 segundos
        self.figures = ['square', 'circle', 'triangle']  # Lista de figuras
        self.index = 0  # Índice para alternar figuras

    def publish_figure(self):
        figure = self.figures[self.index]  # Figura actual
        points = []  # Lista de puntos XYZ

        if figure == 'square':
            # Genera los 4 lados del cuadrado usando 25 puntos por lado
            for i in range(25):
                t = i / 24.0
                points.append([t - 0.5, -0.5, 0.0])           # lado inferior
                points.append([0.5, t - 0.5, 0.0])            # lado derecho
                points.append([0.5 - t, 0.5, 0.0])            # lado superior
                points.append([-0.5, 0.5 - t, 0.0])           # lado izquierdo

        elif figure == 'circle':
            # Círculo con 100 puntos distribuidos uniformemente
            for i in range(100):
                angle = 2 * math.pi * i / 100
                x = math.cos(angle) * 0.5
                y = math.sin(angle) * 0.5
                points.append([x, y, 0.0])

        elif figure == 'triangle':
            # Vértices del triángulo equilátero centrado
            v1 = [0.0, math.sqrt(3)/3]
            v2 = [-0.5, -math.sqrt(3)/6]
            v3 = [0.5, -math.sqrt(3)/6]

            for i in range(34):  # Cada lado con 34 puntos
                t = i / 33.0
                points.append([
                    (1 - t) * v1[0] + t * v2[0],
                    (1 - t) * v1[1] + t * v2[1],
                    0.0
                ])
                points.append([
                    (1 - t) * v2[0] + t * v3[0],
                    (1 - t) * v2[1] + t * v3[1],
                    0.0
                ])
                points.append([
                    (1 - t) * v3[0] + t * v1[0],
                    (1 - t) * v3[1] + t * v1[1],
                    0.0
                ])

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        cloud = pc2.create_cloud_xyz32(header, points)
        self.publisher.publish(cloud)  # Publicar al tópico
        self.get_logger().info(f'Figura publicada: {figure.upper()}')  # Mensaje en el terminal

        self.index = (self.index + 1) % len(self.figures)  # Alternar a la siguiente figura

def main(args=None):
    rclpy.init(args=args)
    node = FigurePublisher()
    rclpy.spin(node)
    rclpy.shutdown()