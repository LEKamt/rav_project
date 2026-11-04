"""Diagnóstico de sensores.

    ros2 launch dron_sensors diagnostico.launch.py

Lo incluyen dron_bringup/simulacion_completa.launch.py y se puede añadir al
arranque de hardware. Para verlo:

    ros2 run rqt_robot_monitor rqt_robot_monitor
    ros2 topic echo /diagnostics
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('dron_sensors')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Usar el reloj de Gazebo'),
        Node(
            package='dron_sensors',
            executable='diagnostico',
            name='diagnostico_sensores',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'limites.yaml'),
                        {'use_sim_time': use_sim_time}],
        ),
    ])
