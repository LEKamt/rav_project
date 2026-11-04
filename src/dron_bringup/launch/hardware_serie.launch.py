"""Dron real por puerto serie: agente micro-ROS + puente + monitor.

    ros2 launch dron_bringup hardware_serie.launch.py dev:=/dev/ttyUSB0

La estación de tierra NO se lanza aquí: necesita un terminal propio para leer el
teclado, y ros2 launch no le da stdin. En otra terminal:

    ros2 run dron_bringup estacion_tierra

El agente vive en un workspace aparte (micro_ros_setup), así que se arranca con
bash -lc sourceando ese workspace.
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

    dev = LaunchConfiguration('dev')
    baud = LaunchConfiguration('baud')
    ws = LaunchConfiguration('ws_microros')
    verbosidad = LaunchConfiguration('v')
    monitor = LaunchConfiguration('monitor')
    tf = LaunchConfiguration('tf')

    return LaunchDescription([
        DeclareLaunchArgument('dev', default_value=cfg['dev'],
                              description='Puerto serie del ESP32-S3'),
        DeclareLaunchArgument('baud', default_value=str(cfg['baud']),
                              description='Debe coincidir con MICROROS_SERIAL_BAUD'),
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
                  'micro_ros_agent serial --dev ', dev, ' -b ', baud, ' -v', verbosidad]],
            output='screen',
        ),

        # Traduce los arrays del firmware a los mensajes de dron_interfaces
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
