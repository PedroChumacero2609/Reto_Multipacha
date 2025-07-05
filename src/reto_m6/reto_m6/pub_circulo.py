import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import math

class CirclePublisher(Node):
    def __init__(self):
        super().__init__('circle_publisher')
        self.publisher = self.create_publisher(PointCloud2, '/reto_pointcloud', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        radius = 1.0
        num_points = 120  # Más puntos = figura más suave
        points = []

        for i in range(num_points):
            angle = (2 * math.pi * i) / num_points
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = 0.0
            points.append([x, y, z])

        cloud = pc2.create_cloud_xyz32(header, points)
        self.publisher.publish(cloud)
        self.get_logger().info(f'Publicando círculo de puntos')

def main(args=None):
    rclpy.init(args=args)
    node = CirclePublisher()
    rclpy.spin(node)
    rclpy.shutdown()