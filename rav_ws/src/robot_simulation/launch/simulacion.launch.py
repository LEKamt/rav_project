"""Simulación del dron F450 en Gazebo Classic.

    ros2 launch robot_simulation simulacion.launch.py
    ros2 launch robot_simulation simulacion.launch.py gui:=false mallas:=false
    ros2 launch robot_simulation simulacion.launch.py control:=false
    ros2 launch robot_simulation simulacion.launch.py ley:=pid

Arranca gzserver (y gzclient si gui:=true), publica el modelo de
robot_description, lo inserta y, salvo que control:=false, incluye el lanzador de
robot_control con el nodo que hace el papel del ESP32-S3.

La estación de tierra va aparte, porque necesita su propio terminal:

    ros2 run robot_bringup estacion_tierra

mallas:=false usa el chasis de primitivas en vez de la malla COLLADA (57 MB que
Gazebo parsea en cada arranque). La física es idéntica; solo cambia el visual.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_sim = get_package_share_directory('robot_simulation')
    pkg_desc = get_package_share_directory('robot_description')
    pkg_control = get_package_share_directory('robot_control')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    with open(os.path.join(pkg_sim, 'config', 'mundo.yaml')) as f:
        cfg = yaml.safe_load(f)['simulacion']

    gui = LaunchConfiguration('gui')
    mallas = LaunchConfiguration('mallas')
    mundo = LaunchConfiguration('mundo')
    altura_inicial = LaunchConfiguration('z')
    control = LaunchConfiguration('control')
    ley = LaunchConfiguration('ley')

    modelo = ParameterValue(
        Command(['xacro ', os.path.join(pkg_desc, 'urdf', 'dron_f450.urdf.xacro'),
                 ' mallas:=', mallas, ' simulacion:=true']),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Lanzar gzclient (interfaz de Gazebo)'),
        DeclareLaunchArgument('mallas', default_value='true',
                              description='Usar las mallas COLLADA en vez de primitivas'),
        DeclareLaunchArgument(
            'mundo', default_value=os.path.join(pkg_sim, 'worlds', cfg['mundo']),
            description='Mundo SDF a cargar'),
        DeclareLaunchArgument('z', default_value=str(cfg['z_inicial']),
                              description='Altura inicial del dron (sobre las patas)'),
        DeclareLaunchArgument('control', default_value='true',
                              description='Incluir el controlador de robot_control'),
        DeclareLaunchArgument('ley', default_value='smc',
                              description='Ley de control: smc o pid'),

        # Para que Gazebo encuentre los modelos propios de models/ y las mallas: el
        # URDF las pide como package://robot_description/... y Gazebo las busca como
        # model://robot_description/..., o sea en el directorio padre del paquete.
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(pkg_sim, 'models') + ':' + os.path.dirname(pkg_desc) + ':' +
            os.environ.get('GAZEBO_MODEL_PATH', '')),

        # gzclient no dibuja nada hasta terminar de consultar el catálogo en línea
        # de modelos, y si el servidor tarda se queda en "Preparing your world".
        # Los modelos que usa el mundo ya están en local.
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': mundo, 'verbose': 'true'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(gui),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': modelo, 'use_sim_time': True}],
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=['-topic', 'robot_description',
                       '-entity', cfg['nombre_entidad'],
                       '-x', str(cfg['x_inicial']),
                       '-y', str(cfg['y_inicial']),
                       '-z', altura_inicial],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_control, 'launch', 'control.launch.py')),
            launch_arguments={'use_sim_time': 'true', 'ley': ley}.items(),
            condition=IfCondition(control),
        ),
    ])
