import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math

class FigureDetector(Node):
    def __init__(self):
        super().__init__('figure_detector')
        self.subscriber = self.create_subscription(
            PointCloud2,
            'figure_cloud',
            self.callback,
            10
        )

    def callback(self, msg):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        num_points = len(points)

        if num_points < 10:
            return

        # Distancia radial desde el centro (0, 0)
        distances = [math.sqrt(x**2 + y**2) for x, y, _ in points]
        avg_radius = sum(distances) / len(distances)
        std_radius = math.sqrt(sum((r - avg_radius) ** 2 for r in distances) / len(distances))

        # Clasificación por desviación estándar del radio
        if std_radius < 0.02:
            figura = "círculo"
        elif std_radius >= 0.08:
            figura = "triángulo"
        else:
            figura = "cuadrado"

        self.get_logger().info(
            f"Detectado: {figura.upper()}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = FigureDetector()
    rclpy.spin(node)
    rclpy.shutdown()