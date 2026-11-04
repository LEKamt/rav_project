"""Todo el sistema en simulación: Gazebo + control + TF + RViz + diagnóstico.

    ros2 launch dron_bringup simulacion_completa.launch.py
    ros2 launch dron_bringup simulacion_completa.launch.py gui:=false rviz:=false

La estación de tierra va en otra terminal (necesita el teclado):

    ros2 run dron_bringup estacion_tierra
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('dron_simulation')
    pkg_loc = get_package_share_directory('dron_localization')
    pkg_sen = get_package_share_directory('dron_sensors')
    pkg_desc = get_package_share_directory('dron_description')

    gui = LaunchConfiguration('gui')
    mallas = LaunchConfiguration('mallas')
    rviz = LaunchConfiguration('rviz')
    diagnostico = LaunchConfiguration('diagnostico')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Interfaz de Gazebo'),
        DeclareLaunchArgument('mallas', default_value='true',
                              description='Mallas COLLADA en vez de primitivas'),
        DeclareLaunchArgument('rviz', default_value='true', description='Lanzar RViz'),
        DeclareLaunchArgument('diagnostico', default_value='true',
                              description='Lanzar el diagnostico de sensores'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_sim, 'launch', 'simulacion.launch.py')),
            launch_arguments={'gui': gui, 'mallas': mallas, 'control': 'true'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_loc, 'launch', 'tf_estado.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_sen, 'launch', 'diagnostico.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
            condition=IfCondition(diagnostico),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_desc, 'rviz', 'dron.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(rviz),
        ),
    ])
