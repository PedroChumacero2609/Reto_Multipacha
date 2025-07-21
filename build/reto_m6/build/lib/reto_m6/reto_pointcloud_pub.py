import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import numpy as np


class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_pub')  # Nombre del nodo
        self.publisher_ = self.create_publisher(PointCloud2, 'reto_pointcloud', 10)  # Creación del publicador
        self.timer = self.create_timer(1.0, self.publish_pointcloud)  # Llama a la función cada 1 segundo
        self.get_logger().info('Publicando nube de puntos en /reto_pointcloud')  # Mensaje de inicio

    def publish_pointcloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        # Nube de puntos: 3 puntos con coordenadas (x, y, z)
        points = np.array([
            [1.0, 0.0, 0.0],  # Punto en eje X
            [0.0, 1.0, 0.0],  # Punto en eje Y
            [0.0, 0.0, 1.0],  # Punto en eje Z
        ], dtype=np.float32)

        # Definimos los campos de cada punto (x, y, z)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Convertimos los puntos a bytes para el mensaje
        data = points.tobytes()

        pointcloud_msg = PointCloud2()
        pointcloud_msg.header = header
        pointcloud_msg.height = 1
        pointcloud_msg.width = points.shape[0]
        pointcloud_msg.fields = fields
        pointcloud_msg.is_bigendian = False
        pointcloud_msg.point_step = 12
        pointcloud_msg.row_step = pointcloud_msg.point_step * points.shape[0]
        pointcloud_msg.is_dense = True
        pointcloud_msg.data = data 

        # Publicamos la nube
        self.publisher_.publish(pointcloud_msg)
        self.get_logger().info('Nube publicada')  # Confirmación en terminal


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
