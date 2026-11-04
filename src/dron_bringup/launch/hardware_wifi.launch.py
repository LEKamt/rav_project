"""Dron real por Wi-Fi UDP: agente micro-ROS + puente + monitor.

    ros2 launch dron_bringup hardware_wifi.launch.py puerto:=8888

El ESP32-S3 entra como estación en la red y abre la sesión contra la IP que
tenga configurada en MICROROS_AGENTE_IP (src/config.h). Esa IP debe ser la del
PC donde corre este agente, en la misma red.

La estación de tierra va en otra terminal:

    ros2 run dron_bringup estacion_tierra
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('dron_bringup')
    pkg_loc = get_package_share_directory('dron_localization')
    with open(os.path.join(pkg, 'config', 'agente.yaml')) as f:
        cfg = yaml.safe_load(f)['agente']

    puerto = LaunchConfiguration('puerto')
    ws = LaunchConfiguration('ws_microros')
    verbosidad = LaunchConfiguration('v')
    monitor = LaunchConfiguration('monitor')
    tf = LaunchConfiguration('tf')

    return LaunchDescription([
        DeclareLaunchArgument('puerto', default_value=str(cfg['puerto_udp']),
                              description='Debe coincidir con MICROROS_AGENTE_PUERTO'),
        DeclareLaunchArgument('ws_microros', default_value=cfg['ws_microros'],
                              description='Workspace con micro_ros_agent compilado'),
        DeclareLaunchArgument('v', default_value=str(cfg['verbosidad']),
                              description='Verbosidad del agente (0-6)'),
        DeclareLaunchArgument('monitor', default_value='true',
                              description='Lanzar el monitor de telemetria'),
        DeclareLaunchArgument('tf', default_value='true',
                              description='Lanzar el publicador de TF/odometria'),

        ExecuteProcess(
            cmd=['bash', '-lc',
                 ['source ', ws, '/install/setup.bash && exec ros2 run micro_ros_agent '
                  'micro_ros_agent udp4 --port ', puerto, ' -v', verbosidad]],
            output='screen',
        ),

        Node(
            package='dron_bringup',
            executable='puente_firmware',
            name='puente_firmware',
            output='screen',
        ),

        Node(
            package='dron_bringup',
            executable='monitor',
            name='monitor_dron',
            output='screen',
            emulate_tty=True,
            parameters=[os.path.join(pkg, 'config', 'agente.yaml')],
            condition=IfCondition(monitor),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_loc, 'launch', 'tf_estado.launch.py')),
            launch_arguments={'use_sim_time': 'false'}.items(),
            condition=IfCondition(tf),
        ),
    ])
