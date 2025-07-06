import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math

class CirclePointDetector(Node):
    def __init__(self):
        super().__init__('circle_point_detector')
        self.subscriber = self.create_subscription(
            PointCloud2,
            'random_circle_cloud',
            self.callback,
            10
        )
        self.radius = 1.0  # mismo que el publicador
        self.epsilon = 0.01  # margen para ignorar puntos en el borde

    def callback(self, msg):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        inside_count = 0
        total = 0

        for x, y, _ in points:
            r = math.sqrt(x**2 + y**2)
            # Ignorar puntos en el borde (±epsilon)
            if abs(r - self.radius) < self.epsilon:
                continue
            if r < self.radius:
                inside_count += 1
            total += 1

        self.get_logger().info(
            f'Puntos totales: {total}, Dentro del círculo (sin borde): {inside_count}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = CirclePointDetector()
    rclpy.spin(node)
    rclpy.shutdown()