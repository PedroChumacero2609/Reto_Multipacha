import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import math
import random

class RandomWithCirclePublisher(Node):
    def __init__(self):
        super().__init__('random_with_circle_publisher')  # Nombre del nodo
        self.publisher = self.create_publisher(PointCloud2, 'random_circle_cloud', 10)  # Publicador del tópico
        self.timer = self.create_timer(5.0, self.publish_cloud)  # Publica cada 5 segundos
        self.circle_radius = 1.0  # Radio del círculo (puede modificarse)

    def publish_cloud(self):
        points = []

        # Genera 200 puntos aleatorios en un área de 4x4 (entre -2 y 2 en x e y)
        for _ in range(200):
            x = random.uniform(-2.0, 2.0)
            y = random.uniform(-2.0, 2.0)
            points.append([x, y, 0.0])  # Todos los puntos están en el plano z=0

        # Añade 100 puntos formando un círculo perfecto en el centro
        for i in range(100):
            angle = 2 * math.pi * i / 100  # Ángulo en radianes para cubrir 360°
            x = math.cos(angle) * self.circle_radius
            y = math.sin(angle) * self.circle_radius
            points.append([x, y, 0.0])

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        # Crea el mensaje PointCloud2 y lo publica
        cloud = pc2.create_cloud_xyz32(header, points)
        self.publisher.publish(cloud)
        self.get_logger().info('Nube publicada con puntos aleatorios y círculo')  # Mensaje en terminal

def main(args=None):
    rclpy.init(args=args) 
    node = RandomWithCirclePublisher() 
    rclpy.spin(node)                
    rclpy.shutdown()              