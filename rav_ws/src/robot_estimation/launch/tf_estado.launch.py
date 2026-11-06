"""Publicador de TF y odometría a partir del estado estimado.

    ros2 launch robot_estimation tf_estado.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_estimation')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Usar el reloj de Gazebo'),
        Node(
            package='robot_estimation',
            executable='tf_estado',
            name='tf_estado',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'estimacion.yaml'),
                        {'use_sim_time': use_sim_time}],
        ),
    ])
