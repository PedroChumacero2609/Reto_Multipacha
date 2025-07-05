from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        FindPackageShare('reto_m6'),
        'urdf',
        'ret_robot.urdf'
    ])

    return LaunchDescription([
        # Publica los estados de las juntas con interfaz gráfica
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'robot_description': urdf_path}]
        ),

        # Publica el modelo del robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': PathJoinSubstitution([
                FindPackageShare('reto_m6'),
                'urdf',
                'ret_robot.urdf'
            ])}]
        ),

        # Lanzar RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d'],
            output='screen'
        )
    ])