from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'reto_m6'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/reto_robot.urdf']),
        ('share/' + package_name + '/launch', ['launch/launch_robot.py', 'launch/display.launch.py', 'launch/description.launch.py']),
        ('share/' + package_name + '/config', ['config/urdf.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pedro2609',
    maintainer_email='pedro2609@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reto_topic_pub = reto_m6.reto_topic_pub:main',
            'reto_topic_sub = reto_m6.reto_topic_sub:main',
            'reto_pointcloud_pub = reto_m6.reto_pointcloud_pub:main',
            'figure_publisher = reto_m6.figure_publisher:main',
            'figure_detector = reto_m6.figure_detector:main',
            'circle_point_publisher = reto_m6.circle_point_publisher:main',
            'circle_point_detector = reto_m6.circle_point_detector:main',
        ],
    },
)