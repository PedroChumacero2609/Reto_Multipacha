import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


# Función de los nodos a lanzar
def launch_setup(context, *args, **kwargs):
    urdf_package = LaunchConfiguration('urdf_package').perform(context)
    urdf_relative_path = LaunchConfiguration('urdf_package_path').perform(context)

    # Ruta completa del paquete y del archivo URDF
    package_path = FindPackageShare(urdf_package).find(urdf_package)
    urdf_file_path = os.path.join(package_path, urdf_relative_path)

    # Leer contenido del archivo URDF para pasarlo como parámetro
    with open(urdf_file_path, 'r') as infp:
        urdf_content = infp.read()

    # Transformaciones del robot
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(urdf_content, value_type=str)
        }]
    )

    # Nodo sin GUI
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('jsp_gui'))  # Solo si jsp_gui == false
    )

    # Nodo con GUI
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp_gui'))  # Solo si jsp_gui == true
    )

    # Nodo del RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],  # Carga configuración RViz
        output='screen'
    )

    # Lista de nodos a lanzar
    return [
        robot_state_publisher_node,
        jsp_node,
        jsp_gui_node,
        rviz_node
    ]


# Función principal
def generate_launch_description():
    return LaunchDescription([
        # Definición de paquetes
        DeclareLaunchArgument(
            name='urdf_package',
            default_value='reto_m6',
            description='Package containing the URDF file'
        ),
        # Ruta relativa al archivo URDF
        DeclareLaunchArgument(
            name='urdf_package_path',
            default_value='urdf/reto_robot.urdf',
            description='Relative path to URDF inside the package'
        ),
        # Argumento para activar o no la GUI del joint_state_publisher
        DeclareLaunchArgument(
            name='jsp_gui',
            default_value='true',
            choices=['true', 'false'],
            description='Use joint_state_publisher_gui if true'
        ),
        # Ruta al archivo de configuración de RViz
        DeclareLaunchArgument(
            name='rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration('urdf_package')),
                'config',
                'urdf.rviz'
            ]),
            description='Path to the RViz configuration file'
        ),
        
        OpaqueFunction(function=launch_setup)
    ])