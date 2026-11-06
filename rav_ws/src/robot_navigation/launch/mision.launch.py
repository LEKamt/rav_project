"""Misiones de altura.

    ros2 launch robot_navigation mision.launch.py                      # solo la accion
    ros2 launch robot_navigation mision.launch.py perfil:=perfil_hover  # perfil completo

Con perfil, el nodo arma el dron y ejecuta la secuencia él solo, así que la
simulación (o el dron) tiene que estar ya en marcha.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perfil = LaunchConfiguration('perfil')
    latido = LaunchConfiguration('latido')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('perfil', default_value='',
                              description='Perfil de param/ a ejecutar (vacio: solo la accion)'),
        DeclareLaunchArgument('latido', default_value='true',
                              description='Publicar /dron/cmd_actitud a 20 Hz'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Usar el reloj de Gazebo'),
        Node(
            package='robot_navigation',
            executable='mision_altura',
            name='mision_altura',
            output='screen',
            emulate_tty=True,
            parameters=[{'perfil': perfil, 'publicar_latido': latido,
                         'use_sim_time': use_sim_time}],
        ),
    ])
