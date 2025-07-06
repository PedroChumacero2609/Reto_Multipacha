import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import math
import random

class RandomWithCirclePublisher(Node):
    def __init__(self):
        super().__init__('random_with_circle_publisher')
        self.publisher = self.create_publisher(PointCloud2, 'random_circle_cloud', 10)
        self.timer = self.create_timer(5.0, self.publish_cloud)
        self.circle_radius = 1.0  # puedes cambiarlo

    def publish_cloud(self):
        points = []

        # Generar 200 puntos aleatorios en [-2, 2] x [-2, 2]
        for _ in range(200):
            x = random.uniform(-2.0, 2.0)
            y = random.uniform(-2.0, 2.0)
            points.append([x, y, 0.0])

        # Añadir círculo de borde (100 puntos en el perímetro)
        for i in range(100):
            angle = 2 * math.pi * i / 100
            x = math.cos(angle) * self.circle_radius
            y = math.sin(angle) * self.circle_radius
            points.append([x, y, 0.0])  # Borde del círculo

        # Crear nube y publicarla
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        cloud = pc2.create_cloud_xyz32(header, points)
        self.publisher.publish(cloud)
        self.get_logger().info('Nube publicada con puntos aleatorios y círculo')

def main(args=None):
    rclpy.init(args=args)
    node = RandomWithCirclePublisher()
    rclpy.spin(node)
    rclpy.shutdown()