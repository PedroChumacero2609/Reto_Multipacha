from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Argumentos
    declared_arguments = [
        DeclareLaunchArgument('jsp_gui', default_value='true',
                              choices=['true', 'false'], description='Usar joint_state_publisher_gui'),
        DeclareLaunchArgument('urdf_package', default_value='reto_m6',
                              description='Paquete que contiene el URDF'),
        DeclareLaunchArgument('urdf_package_path', default_value='urdf/reto_robot.urdf',
                              description='Ruta al archivo URDF o XACRO dentro del paquete'),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([
            FindPackageShare('reto_m6'), 'config', 'urdf.rviz'
        ]), description='Archivo de configuración RViz')
    ]

    # Ruta del archivo URDF o XACRO
    urdf_pkg = FindPackageShare(LaunchConfiguration('urdf_package'))
    urdf_file = PathJoinSubstitution([urdf_pkg, LaunchConfiguration('urdf_package_path')])
    robot_description_content = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # Nodos a lanzar
    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(LaunchConfiguration('jsp_gui')),
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('jsp_gui')),
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen'
        )
    ]

    return LaunchDescription(declared_arguments + nodes)