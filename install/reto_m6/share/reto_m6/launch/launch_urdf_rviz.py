from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('reto_robot'), 'launch', 'display.launch.py'
                ])
            ),
            launch_arguments={
                'urdf_package': 'reto_robot',
                'urdf_package_path': PathJoinSubstitution([
                    FindPackageShare('reto_robot'), 'urdf', 'reto_robot.urdf'
                ])
            }.items()
        )
    ])