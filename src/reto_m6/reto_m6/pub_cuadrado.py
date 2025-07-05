import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class SquarePublisher(Node):
    def __init__(self):
        super().__init__('square_publisher')
        self.publisher = self.create_publisher(PointCloud2, '/reto_pointcloud', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        num_points_per_side = 25
        points = []

        # Lado inferior (0,0) -> (1,0)
        for i in np.linspace(0, 1, num_points_per_side):
            points.append([i, 0.0, 0.0])
        # Lado derecho (1,0) -> (1,1)
        for i in np.linspace(0, 1, num_points_per_side):
            points.append([1.0, i, 0.0])
        # Lado superior (1,1) -> (0,1)
        for i in np.linspace(0, 1, num_points_per_side):
            points.append([1.0 - i, 1.0, 0.0])
        # Lado izquierdo (0,1) -> (0,0)
        for i in np.linspace(0, 1, num_points_per_side):
            points.append([0.0, 1.0 - i, 0.0])

        cloud = pc2.create_cloud_xyz32(header, points)
        self.publisher.publish(cloud)
        self.get_logger().info(f'Publicando puntos en forma de cuadrado')

def main(args=None):
    rclpy.init(args=args)
    node = SquarePublisher()
    rclpy.spin(node)
    rclpy.shutdown()