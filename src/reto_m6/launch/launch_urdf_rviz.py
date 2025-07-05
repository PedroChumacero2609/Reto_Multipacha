from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare
import os

def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('reto_m6').perform(context)
    urdf_path = os.path.join(pkg_share, 'urdf', 'reto_robot.urdf')

    # Lee el contenido del archivo URDF
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    return [
        # GUI para mover juntas
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Publica el modelo del robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Lanzar RViz sin archivo -d
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])