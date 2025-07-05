import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math

class ShapeDetector(Node):
    def __init__(self):
        super().__init__('shape_detector')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/reto_pointcloud',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append((p[0], p[1]))

        if len(points) < 10:
            self.get_logger().info("⛔ No hay suficientes puntos para detectar forma.")
            return

        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)

        distances = [math.hypot(x - cx, y - cy) for x, y in points]
        min_d = min(distances)
        max_d = max(distances)
        diff = max_d - min_d

        # Regla simple para determinar forma
        if diff < 0.1:
            self.get_logger().info("⚪ Se detectó un CÍRCULO")
        elif 30 <= len(points) <= 40:
            self.get_logger().info("🟩 Se detectó un CUADRADO")
        else:
            self.get_logger().info("❓ No se detectó una figura geométrica")

def main(args=None):
    rclpy.init(args=args)
    node = ShapeDetector()
    rclpy.spin(node)
    rclpy.shutdown()