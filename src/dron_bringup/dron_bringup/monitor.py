#!/usr/bin/env python3
"""Monitor no interactivo: imprime la telemetría en una línea por muestra.

Útil para registrar sesiones de banco de pruebas junto con `ros2 bag record`, o
para ver qué llega sin tomar el control del terminal con la estación.
"""

import rclpy
from dron_interfaces.msg import EstadoAltura, EstadoDron, SenalesControl
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class Monitor(Node):

    def __init__(self):
        super().__init__('monitor_dron')

        self.declare_parameter('frecuencia', 5.0)
        frecuencia = self.get_parameter('frecuencia').value

        self.alt = None
        self.sen = None
        self.estado = None

        self.create_subscription(EstadoAltura, 'dron/estado_altura', self.cb_altura,
                                 qos_profile_sensor_data)
        self.create_subscription(SenalesControl, 'dron/senales_control', self.cb_senales,
                                 qos_profile_sensor_data)
        self.create_subscription(EstadoDron, 'dron/estado_dron', self.cb_estado, 10)
        self.create_timer(1.0 / frecuencia, self.imprimir)

    def cb_altura(self, msg):
        self.alt = msg

    def cb_senales(self, msg):
        self.sen = msg

    def cb_estado(self, msg):
        self.estado = msg

    def imprimir(self):
        if self.alt is None or self.sen is None:
            self.get_logger().info('esperando telemetria...')
            return
        a, s = self.alt, self.sen
        modo = self.estado.descripcion if self.estado else '?'
        self.get_logger().info(
            '%-22s h=%.2f (obj %.2f, lidar %.2f%s, baro %.2f) vz=%+.2f '
            'thr=%.3f vz_sp=%+.2f a_sp=%+.2f corr=(%+.3f %+.3f %+.3f)'
            % (modo, a.fusionada, a.objetivo, a.lidar, '' if a.lidar_valido else '!',
               a.baro, a.velocidad_vertical, s.throttle,
               s.velocidad_vertical_deseada, s.aceleracion_vertical_deseada,
               s.correccion_roll, s.correccion_pitch, s.correccion_yaw))


def main():
    rclpy.init()
    nodo = Monitor()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
