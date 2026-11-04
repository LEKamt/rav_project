"""Controlador de vuelo (el que hace el papel del ESP32-S3 en simulación).

    ros2 launch dron_control control.launch.py

Normalmente no se lanza solo: dron_simulation/simulacion.launch.py lo incluye.
Las ganancias salen de config/ganancias.yaml y el modelo de planta de
config/parametros_sim.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('dron_control')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Usar el reloj de Gazebo'),
        Node(
            package='dron_control',
            executable='controlador_sim',
            name='controlador_sim',
            output='screen',
            emulate_tty=True,
            parameters=[
                os.path.join(pkg, 'config', 'parametros_sim.yaml'),
                os.path.join(pkg, 'config', 'ganancias.yaml'),
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
