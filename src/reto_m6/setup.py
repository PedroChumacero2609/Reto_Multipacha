from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'reto_m6'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
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
            'pub_cuadrado = reto_m6.pub_cuadrado:main',
            'pub_circulo = reto_m6.pub_circulo:main',
            'pcl_shape_detector = reto_m6.pcl_shape_detector:main',
        ],
    },
)
