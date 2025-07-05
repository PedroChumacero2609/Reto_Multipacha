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