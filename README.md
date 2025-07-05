# Reto Multipacha

## Instalación

### Requisitos Previos
- Sistema operativo: Ubuntu 22.04 (o superior)
- Visual Studio Code
- ROS2 Jazzy Jalisco instalado
- Dependencias de sistema operativo estándar para desarrollo de ROS2
- RVIZ2

### Pasos para instalación de ROS2 Jazzy

Una vez instalado Ubuntu 22.04 y el editor Visual Studio Code, procedí a instalar **ROS2 Jazzy** y sus herramientas necesarias siguiendo estos pasos:

1. Configuré el sistema y la codificación UTF-8:

```bash
locale  # Comprobar si ya se usa UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

2. Habilité los repositorios necesarios:
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
3. Añadí el repositorio de ROS2:
```bash
sudo apt update && sudo apt install curl -y
```
```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```
4. Instalé herramientas de desarrollo para ROS2:
```bash
sudo apt update
sudo apt install ros-dev-tools
```
5. Instalé ROS2 Jazzy:
```bash
sudo apt update
sudo apt upgrade
```
```bash
sudo apt install ros-jazzy-desktop
```
```bash
sudo apt install ros-jazzy-ros-base
```
6. Configuré entorno de ROS2:
```bash
source /opt/ros/jazzy/setup.bash
```
## Desarrollo en ROS

## Parte 1
### Manejo de Directorios y Paquetes

Para comenzar con el desarrollo del proyecto, primero preparé el espacio de trabajo de ROS2 y creé la estructura básica del paquete que utilicé durante el desarrollo.

1. Abrí una terminal normal para asegurarme de estar en mi directorio personal:
```bash
cd ~
```
2. Luego, creé el espacio de trabajo multipacha_ws con su carpeta src y abrí Visual Studio Code directamente en esa ruta:
```bash
mkdir -p multipacha_ws/src
cd multipacha_ws
code .
```
3. Una vez dentro de VS Code, me moví al directorio src para crear el paquete llamado reto_m6 con las dependencias necesarias rclpy y rviz2:
```bash
cd src
ros2 pkg create reto_m6 --build-type ament_python --dependencies rclpy rviz2
```
4. Exitosamente, el paquete se creó correctamente en la ruta ~/multipacha_ws/src/reto_m6.
5. Después, ingresé al directorio del paquete para crear las carpetas necesarias para los archivos de lanzamiento y el modelo del robot:
```bash
cd reto_m6
mkdir launch urdf
```
### Tópicos publicadores y suscriptores
### Creación del Publicador
1. Creé el archivo Python reto_topic_pub.py dentro del subdirectorio del paquete que contiene los scripts (es decir, dentro de reto_m6/reto_m6/):
```bash
cd reto_m6/reto_m6
touch reto_topic_pub.py
chmod +x reto_topic_pub.py
```
2. Luego, desarrollé un script que actúa como publicador de mensajes en ROS2. El objetivo era que publicara un mensaje de tipo `std_msgs/String` con el texto "M6 Reto" a una frecuencia de 10 Hz. Para eso, edité el archivo ubicado en `~/multipacha_ws/src/reto_m6/reto_m6/reto_topic_pub.py` y escribí el siguiente código:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RetoPublisher(Node):
    def __init__(self):
        super().__init__('reto_topic_pub')
        self.publisher_ = self.create_publisher(String, 'reto_topic', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz = 0.1 s
        self.get_logger().info('Publicador iniciado a 10 Hz')

    def timer_callback(self):
        msg = String()
        msg.data = 'M6 Reto'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicando: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    nodo = RetoPublisher()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
3. Después, me aseguré de que el archivo fuera ejecutable:
```bash
chmod +x ~/multipacha_ws/src/reto_m6/reto_m6/reto_topic_pub.py
```
4. A continuación, edité el archivo setup.py del paquete para registrar el script como ejecutable desde ROS2. Dentro del archivo, modifiqué la siguiente sección:
```python
entry_points={
    'console_scripts': [
    ],
},
```
  Por esta forma:
```python
entry_points={
    'console_scripts': [
        'reto_topic_pub = reto_m6.reto_topic_pub:main',
    ],
},
```
5. Guardé los cambios y volví a compilar el workspace:
```bash
cd ~/multipacha_ws
colcon build
source install/setup.bash
```
6. Finalmente, ejecuté mi nodo publicador con el siguiente comando:
```bash
ros2 run reto_m6 reto_topic_pub
```
6. Como resultado, observé en la terminal la publicación continua del mensaje "M6 Reto" a 10 Hz:
![Publicador y Suscriptor de Tópicos](./docs/img/pub.png)
Esto confirmó que el nodo reto_topic_pub funcionaba correctamente. 

### Creación del Suscriptor
1. Creé el archivo Python `reto_topic_sub.py` dentro del subdirectorio del paquete que contiene los scripts (es decir, dentro de `reto_m6/reto_m6/`):
```bash
cd reto_m6/reto_m6
touch reto_topic_sub.py
chmod +x reto_topic_sub.py
```
2. Luego, desarrollé un script que actúa como suscriptor de mensajes en ROS2. El objetivo era que escuchara el tópico reto_topic y mostrara los mensajes que se publican. Para eso, edité el archivo ubicado en ~/multipacha_ws/src/reto_m6/reto_m6/reto_topic_sub.py y escribí el siguiente código:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RetoSubscriber(Node):
    def __init__(self):
        super().__init__('reto_topic_sub')
        self.subscription = self.create_subscription(
            String,
            'reto_topic',
            self.listener_callback,
            10)
        self.subscription  # evitar warning por variable sin usar
        self.get_logger().info('Suscriptor iniciado, escuchando...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Recibido: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    nodo = RetoSubscriber()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
3. Me aseguré de que el archivo fuera ejecutable:
```bash
chmod +x ~/multipacha_ws/src/reto_m6/reto_m6/reto_topic_sub.py
```
4. Luego, edité el archivo setup.py del paquete para registrar también este script como ejecutable:
```python
entry_points={
    'console_scripts': [
        'reto_topic_pub = reto_m6.reto_topic_pub:main',
        'reto_topic_sub = reto_m6.reto_topic_sub:main',
    ],
},
```
5. Guardé los cambios y compilé nuevamente el workspace:
```bash
cd ~/multipacha_ws
colcon build
source install/setup.bash
```
6. Finalmente, ejecuté mi nodo suscriptor con el siguiente comando:
```bash
ros2 run reto_m6 reto_topic_sub
```
7. Como resultado, pude observar en el terminal de la derecha cómo el nodo suscriptor recibía y mostraba el mensaje "M6 Reto" publicado por el nodo publicador del terminal de la izquierda:
![Publicador y Suscriptor de Tópicos](./docs/img/sub.png)
Esto confirmó que el nodo reto_topic_sub estaba funcionando correctamente y comunicándose con el nodo reto_topic_pub.

## Modelado URDF
### Creación del modelo básico del robot `reto_robot`
Para esta parte del reto, el objetivo fue crear un modelo simple en formato URDF para representar un robot con al menos dos enlaces (links) y una articulación de tipo revolute o continuous.
1. Primero, dentro del paquete `reto_m6`, navegué al directorio `urdf`.
2. Luego, creé el archivo reto_robot.urdf desde Visual Studio Code.
3. Dentro de este archivo, definí el siguiente modelo URDF:
```xml
<?xml version="1.0"?>
<robot name="reto_robot" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- Enlace base del robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.4 0.1"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Enlace del brazo -->
  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Articulación entre la base y el brazo -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="10.0" velocity="1.0" lower="-3.14" upper="3.14"/>
  </joint>

</robot>
```
### Detalles del modelo
base_link: representa la base del robot. Es una caja azul de 40 x 40 x 10 cm.
arm_link: es un brazo en forma de cilindro rojo de 5 cm de radio y 50 cm de largo, montado verticalmente sobre la base.
base_to_arm: es una articulación de tipo revolute, lo que permite que el brazo gire sobre el eje Z. Esta unión conecta los dos enlaces.

## Archivos Launch
Para facilitar la ejecución de los nodos y la visualización del robot en RViz, creé tres archivos launch dentro del directorio launch del paquete reto_m6.
### Launch del nodo reto_topic_pub.py
1. Creé el archivo Launch del nodo reto_topic_pub.py. Fui a la carpeta reto_m6/launch y creé un archivo llamado launch_pub.py.
2. Dentro del archivo escribí el siguiente código, que lanza el nodo publicador:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='reto_m6',
            executable='reto_topic_pub',
            name='publisher_node'
        )
    ])
```
### Launch del nodo reto_topic_sub.py
1. Creé el archivo Launch del nodo reto_topic_sub.py. Fui a la capeta reto_m6/launch y creé el archivo llamado launch_sub.py.
2. Dentro del archivo escribí el siguiente código:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='reto_m6',
            executable='reto_topic_sub',
            name='subscriber_node',
            output='screen'  # Para ver los mensajes en la terminal
        )
    ])
```
### Launch del nodo reto_robot.urdf
1. Creé el archivo Launch del modelo URDF en RViz con joint_state_publisher_gui. El archivo lo llamé launch_urdf_rviz.py en la misma carpeta launch:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        FindPackageShare('reto_m6'),
        'urdf',
        'ret_robot.urdf'
    ])

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'robot_description': urdf_path}]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': urdf_path}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d'],
            output='screen'
        )
    ])
```
2. Este archivo lanza:
-La GUI para mover la junta del robot.
-El modelo URDF con robot_state_publisher.
-La interfaz RViz2 para visualizar el robot en 3D.
### Compilación de los archivos
Después de crear los archivos Launch, regresé a la raíz del workspace y compilé todo:
```bash
cd ~/multipacha_ws
colcon build
source install/setup.bash
```
Ejecutar el publicador:
```bash
ros2 launch reto_m6 launch_pub.py
```
Ejecutar el suscriptor:
```bash
ros2 launch reto_m6 launch_sub.py
```
Visualizar el robot en RViz2 y manipular la articulación:
```bash
ros2 launch reto_m6 launch_urdf_rviz.py
```
Al lanzar el último archivo, se abrió la interfaz de RViz y la ventana del joint_state_publisher_gui, donde pude mover la junta del robot:
![Publicador y Suscriptor de Tópicos](./docs/img/Junta.png)

## Parte 2
### ¿Qué es sensor_msgs/PointCloud2?
El mensaje sensor_msgs/PointCloud2 es un tipo de mensaje utilizado en ROS 2 para representar nubes de puntos 3D, que normalmente provienen de sensores como LIDAR o cámaras RGB-D. Cada punto en la nube contiene al menos coordenadas x, y, z, y, opcionalmente, otros campos como rgb, intensidad, etc. Este mensaje es muy eficiente ya que empaqueta los datos como un arreglo binario, con su estructura (campos, tipo, offset, etc.), permitiendo representar miles de puntos sin perder rendimiento.

### Creación del publicador reto_pointcloud_pub.py
1. Creé el archivo dentro del subdirectorio del paquete:
```bash
cd ~/multipacha_ws/src/reto_m6/reto_m6
touch reto_pointcloud_pub.py
chmod +x reto_pointcloud_pub.py
```
2. Escribí el siguiente código para publicar una nube de 3 puntos simples:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_pub')
        self.publisher_ = self.create_publisher(PointCloud2, 'reto_pointcloud', 10)
        self.timer = self.create_timer(1.0, self.publish_pointcloud)  # 1 Hz
        self.get_logger().info('Publicando nube de puntos en /reto_pointcloud')

    def publish_pointcloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'  # Necesario para RViz

        # Creamos 3 puntos 3D
        points = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float32)

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        data = points.tobytes()
        pointcloud_msg = PointCloud2()
        pointcloud_msg.header = header
        pointcloud_msg.height = 1
        pointcloud_msg.width = points.shape[0]
        pointcloud_msg.fields = fields
        pointcloud_msg.is_bigendian = False
        pointcloud_msg.point_step = 12
        pointcloud_msg.row_step = 12 * points.shape[0]
        pointcloud_msg.is_dense = True
        pointcloud_msg.data = data

        self.publisher_.publish(pointcloud_msg)
        self.get_logger().info('Nube publicada')


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
```
3. Actualicé el archivo setup.py para que este nuevo script también pueda ejecutarse desde ROS 2:
```python
entry_points={
    'console_scripts': [
        'reto_topic_pub = reto_m6.reto_topic_pub:main',
        'reto_topic_sub = reto_m6.reto_topic_sub:main',
        'reto_pointcloud_pub = reto_m6.reto_pointcloud_pub:main',
    ],
},
```
4. Una vez agregado el script y actualizado setup.py, volví a compilar todo el workspace:
```bash
cd ~/multipacha_ws
colcon build
source install/setup.bash
```
5. Ejecuté el nodo con el comando:
```bash
ros2 run reto_m6 reto_pointcloud_pub
```
En la terminal pude ver los logs confirmando la publicación del mensaje:
![Publicador y Suscriptor de Tópicos](./docs/img/3_puntos_codigo.png)

### Visualización en Rviz2
```bash
rviz2
```
![Publicador y Suscriptor de Tópicos](./docs/img/3_puntos_grafica.png)
-Cambié el Fixed Frame a map (importante para ver los datos correctamente).
-Hice clic en "Add" → Seleccioné "PointCloud2" → OK
-En el campo Topic, seleccioné /reto_pointcloud.

## Desarrollo para PointCloud
## Parte 1

## Parte 2
