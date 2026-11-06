#!/usr/bin/env python3
"""Puente entre el firmware y los mensajes propios del proyecto.

El ESP32-S3 publica la telemetría en Float32MultiArray y el modo en un Int32,
porque meter mensajes propios en micro-ROS obliga a reconstruir la librería con
extra_packages en los dos entornos. Este nodo traduce esos arrays a los mensajes
de robot_interfaces, que son los que consumen la estación, el diagnóstico y las
misiones. Así hay un solo formato aguas abajo, igual en simulación que con
hardware.

Solo hace falta con el dron real: en simulación, robot_control ya publica los
mensajes propios directamente.

    /dron/alturas  (Float32MultiArray[6]) -> /dron/estado_altura
    /dron/control  (Float32MultiArray[8]) \\
    /dron/motores  (Float32MultiArray[4]) -> /dron/senales_control
    /dron/estado   (Int32)                -> /dron/estado_dron
                                          -> servicio /dron/armar_srv

El servicio de armado publica en /dron/armar y espera a ver el cambio de modo:
el firmware rechaza el armado si no hay altura fijada o si el pulsador de
emergencia está pulsado, y por tópico eso no se ve.
"""

import time

import rclpy
from robot_interfaces.msg import EstadoAltura, EstadoDron, SenalesControl
from robot_interfaces.srv import Armar
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32MultiArray, Int32

NOMBRE_MODO = {0: 'DESARMADO', 1: 'ARMADO', 2: 'ATERRIZAJE', 3: 'DETENIDO'}
NOMBRE_MOTIVO = {0: '', 1: 'boton fisico', 2: 'timeout de comandos',
                 3: 'agente perdido', 4: 'orden por ROS', 5: 'IMU sin lecturas'}

ESPERA_ARMADO_S = 1.5


class PuenteFirmware(Node):

    def __init__(self):
        super().__init__('puente_firmware')

        self.pub_altura = self.create_publisher(EstadoAltura, 'dron/estado_altura',
                                                qos_profile_sensor_data)
        self.pub_senales = self.create_publisher(SenalesControl, 'dron/senales_control',
                                                 qos_profile_sensor_data)
        self.pub_estado = self.create_publisher(EstadoDron, 'dron/estado_dron', 10)
        self.pub_armar = self.create_publisher(Bool, 'dron/armar', 10)

        # El servicio de armado espera a ver el cambio de modo, y mientras espera
        # las suscripciones tienen que seguir corriendo: de ahí el ejecutor
        # multihilo y los grupos separados.
        grupo_sensores = ReentrantCallbackGroup()
        grupo_servicio = MutuallyExclusiveCallbackGroup()

        self.create_subscription(Float32MultiArray, 'dron/alturas', self.cb_alturas,
                                 qos_profile_sensor_data, callback_group=grupo_sensores)
        self.create_subscription(Float32MultiArray, 'dron/control', self.cb_control,
                                 qos_profile_sensor_data, callback_group=grupo_sensores)
        self.create_subscription(Float32MultiArray, 'dron/motores', self.cb_motores,
                                 qos_profile_sensor_data, callback_group=grupo_sensores)
        self.create_subscription(Int32, 'dron/estado', self.cb_estado, 10,
                                 callback_group=grupo_sensores)

        self.srv_armar = self.create_service(Armar, 'dron/armar_srv', self.cb_srv_armar,
                                             callback_group=grupo_servicio)

        self.motores = [0.0] * 4
        self.modo = 0
        self.motivo = 0
        # El firmware no publica si tiene setpoint fijado: se deduce del campo 6
        # de /dron/alturas, que es la altura objetivo aceptada (0 si no hay).
        self.setpoint_fijado = False

        self.get_logger().info('puente_firmware listo (arrays del ESP32 -> robot_interfaces)')

    # ---------- traducción ----------
    def cb_alturas(self, msg):
        if len(msg.data) < 6:
            return
        d = msg.data
        self.setpoint_fijado = d[5] > 0.0

        out = EstadoAltura()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.fusionada, out.lidar, out.baro = float(d[0]), float(d[1]), float(d[2])
        out.consigna = float(d[3])
        out.velocidad_vertical = float(d[4])
        out.objetivo = float(d[5])
        out.lidar_valido = d[1] > 0.0
        self.pub_altura.publish(out)

    def cb_control(self, msg):
        if len(msg.data) < 8:
            return
        d = msg.data
        out = SenalesControl()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.throttle = float(d[0])
        out.correccion_roll, out.correccion_pitch = float(d[1]), float(d[2])
        out.correccion_yaw = float(d[3])
        out.velocidad_vertical_deseada = float(d[4])
        out.aceleracion_vertical_deseada = float(d[5])
        out.rate_deseado_roll, out.rate_deseado_pitch = float(d[6]), float(d[7])
        out.motores = [float(v) for v in self.motores]
        self.pub_senales.publish(out)

    def cb_motores(self, msg):
        if len(msg.data) >= 4:
            self.motores = list(msg.data[:4])

    def cb_estado(self, msg):
        self.modo = msg.data % 10
        self.motivo = msg.data // 10

        out = EstadoDron()
        out.header.stamp = self.get_clock().now().to_msg()
        out.modo = int(self.modo)
        out.motivo_failsafe = int(self.motivo)
        out.setpoint_fijado = bool(self.setpoint_fijado)
        texto = NOMBRE_MODO.get(self.modo, '?')
        if self.motivo:
            texto += ' por ' + NOMBRE_MOTIVO.get(self.motivo, '?')
        out.descripcion = texto
        self.pub_estado.publish(out)

    # ---------- servicio de armado ----------
    def cb_srv_armar(self, peticion, respuesta):
        if peticion.armar and not self.setpoint_fijado:
            respuesta.aceptado = False
            respuesta.motivo = 'falta fijar la altura objetivo'
            return respuesta

        modo_antes = self.modo
        self.pub_armar.publish(Bool(data=bool(peticion.armar)))

        # Espera corta a que el firmware publique el cambio de modo. No se puede
        # llamar a spin_once aquí (sería reentrar en el ejecutor): se duerme, y
        # cb_estado actualiza self.modo desde el otro grupo de callbacks.
        limite = time.monotonic() + ESPERA_ARMADO_S
        while time.monotonic() < limite:
            time.sleep(0.05)
            if self.modo != modo_antes:
                break

        esperado = 1 if peticion.armar else 0
        respuesta.aceptado = (self.modo == esperado)
        if respuesta.aceptado:
            respuesta.motivo = ''
        elif self.modo == modo_antes:
            respuesta.motivo = ('el firmware no cambio de modo (sin respuesta en %.1f s)'
                                % ESPERA_ARMADO_S)
        else:
            respuesta.motivo = 'el dron quedo en %s' % NOMBRE_MODO.get(self.modo, '?')
        return respuesta


def main():
    rclpy.init()
    nodo = PuenteFirmware()
    ejecutor = MultiThreadedExecutor(num_threads=2)
    ejecutor.add_node(nodo)
    try:
        ejecutor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ejecutor.shutdown()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
