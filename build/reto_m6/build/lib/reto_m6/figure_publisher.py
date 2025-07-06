import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import math

class FigurePublisher(Node):
    def __init__(self):
        super().__init__('figure_publisher')
        self.publisher = self.create_publisher(PointCloud2, 'figure_cloud', 10)
        self.timer = self.create_timer(5.0, self.publish_figure)
        self.figures = ['square', 'circle', 'triangle']
        self.index = 0

    def publish_figure(self):
        figure = self.figures[self.index]
        points = []

        if figure == 'square':
            for i in range(25):
                t = i / 24.0
                points.append([t - 0.5, -0.5, 0.0])           # bottom
                points.append([0.5, t - 0.5, 0.0])            # right
                points.append([0.5 - t, 0.5, 0.0])            # top
                points.append([-0.5, 0.5 - t, 0.0])           # left

        elif figure == 'circle':
            for i in range(100):
                angle = 2 * math.pi * i / 100
                x = math.cos(angle) * 0.5
                y = math.sin(angle) * 0.5
                points.append([x, y, 0.0])

        elif figure == 'triangle':
            # Vértices de triángulo equilátero centrado
            v1 = [0.0, math.sqrt(3)/3]
            v2 = [-0.5, -math.sqrt(3)/6]
            v3 = [0.5, -math.sqrt(3)/6]

            for i in range(34):
                t = i / 33.0
                # Lado 1
                points.append([
                    (1 - t) * v1[0] + t * v2[0],
                    (1 - t) * v1[1] + t * v2[1],
                    0.0
                ])
                # Lado 2
                points.append([
                    (1 - t) * v2[0] + t * v3[0],
                    (1 - t) * v2[1] + t * v3[1],
                    0.0
                ])
                # Lado 3
                points.append([
                    (1 - t) * v3[0] + t * v1[0],
                    (1 - t) * v3[1] + t * v1[1],
                    0.0
                ])

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        cloud = pc2.create_cloud_xyz32(header, points)
        self.publisher.publish(cloud)
        self.get_logger().info(f'Figura publicada: {figure.upper()}')

        self.index = (self.index + 1) % len(self.figures)

def main(args=None):
    rclpy.init(args=args)
    node = FigurePublisher()
    rclpy.spin(node)
    rclpy.shutdown()