#!/usr/bin/env python3
"""Estación de tierra del dron F450.

Publica un flujo continuo de /dron/cmd_actitud a 20 Hz: ese flujo es también el
latido del enlace. Si se corta más de TIMEOUT_CMD_MS, el firmware inicia el
aterrizaje por failsafe. Por eso este nodo no debe pararse con el dron en el aire.

Sirve igual para la simulación y para el dron real: consume los mensajes de
robot_interfaces, que en simulación publica robot_control y con hardware produce
`puente_firmware` a partir de los arrays del ESP32.

Teclas:
    a           armar (requiere altura objetivo fijada)
    d           desarmar
    ESPACIO     emergencia -> aterrizaje controlado
    w / s       altura objetivo +/- 0.1 m
    i / k       pitch  adelante / atras
    j / l       roll   izquierda / derecha
    q / e       guiñada izquierda / derecha
    x           centrar actitud
    g           enviar ganancias de actitud (solo con el dron desarmado)
    h           ayuda
    Ctrl-C      salir
"""

import math
import os
import select
import sys
import termios
import tty

import rclpy
from robot_interfaces.msg import EstadoAltura, EstadoDron, SenalesControl
from robot_interfaces.srv import Armar
from geometry_msgs.msg import Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Float32MultiArray

FREQ_CMD = 20.0          # Hz del flujo de comandos (latido)
FREQ_PANTALLA = 5.0      # Hz de refresco del panel

PASO_ALTURA = 0.1        # m
PASO_ANGULO = 2.0        # grados por pulsación
PASO_YAW = 15.0          # deg/s por pulsación
DECAIMIENTO = 0.85       # las consignas de actitud vuelven solas al centro

ALTURA_MAX = 3.0
ANGULO_MAX = 15.0
YAW_MAX = 90.0

MODOS = {0: 'DESARMADO', 1: 'ARMADO', 2: 'ATERRIZAJE', 3: 'DETENIDO'}

# Mismo orden que espera cb_ganancias en el firmware
GANANCIAS_DEF = [4.5, 0.0020, 0.0040, 0.00006, 0.0040, 0.0060]


class EstacionTierra(Node):

    def __init__(self):
        super().__init__('estacion_tierra')

        # Comandos. cmd_actitud va best effort (flujo de alta frecuencia); el
        # resto fiable, porque armar o desarmar no se puede perder.
        self.pub_actitud = self.create_publisher(Vector3, 'dron/cmd_actitud',
                                                 qos_profile_sensor_data)
        self.pub_altura = self.create_publisher(Float32, 'dron/cmd_altura', 10)
        self.pub_armar = self.create_publisher(Bool, 'dron/armar', 10)
        self.pub_emergencia = self.create_publisher(Bool, 'dron/emergencia', 10)
        self.pub_ganancias = self.create_publisher(Float32MultiArray,
                                                   'dron/cmd_ganancias', 10)

        # Armado con confirmación si hay servicio; si no, por tópico
        self.cli_armar = self.create_client(Armar, 'dron/armar_srv')

        # Telemetría. Los mensajes de estado van best effort salvo el de modo.
        self.create_subscription(EstadoAltura, 'dron/estado_altura', self.cb_altura,
                                 qos_profile_sensor_data)
        self.create_subscription(SenalesControl, 'dron/senales_control',
                                 self.cb_senales, qos_profile_sensor_data)
        self.create_subscription(Imu, 'dron/imu', self.cb_imu, qos_profile_sensor_data)
        self.create_subscription(EstadoDron, 'dron/estado_dron', self.cb_estado, 10)

        # Consignas
        self.altura_objetivo = 1.0
        self.altura_enviada = False
        self.roll_sp = 0.0
        self.pitch_sp = 0.0
        self.yaw_rate_sp = 0.0

        # Telemetría recibida
        self.alt = None
        self.sen = None
        self.rpy = (0.0, 0.0, 0.0)
        self.estado = None
        self.ultima_descripcion = None
        self.mensajes = []

        self.create_timer(1.0 / FREQ_CMD, self.enviar_comandos)
        self.create_timer(1.0 / FREQ_PANTALLA, self.pintar)

    # ---------- telemetría ----------
    def cb_altura(self, msg):
        self.alt = msg

    def cb_senales(self, msg):
        self.sen = msg

    def cb_imu(self, msg):
        q = msg.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.degrees(math.atan2(sinr, cosr))
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        pitch = math.degrees(math.asin(sinp))
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.degrees(math.atan2(siny, cosy))
        self.rpy = (roll, pitch, yaw)

    def cb_estado(self, msg):
        self.estado = msg
        if msg.descripcion != self.ultima_descripcion:
            self.ultima_descripcion = msg.descripcion
            self.aviso('estado: ' + msg.descripcion)
        # El firmware borra su setpoint al desarmar y al terminar un aterrizaje:
        # el panel debe pedir que se fije otra vez, no mentir con el último valor.
        if not msg.setpoint_fijado:
            self.altura_enviada = False

    def aviso(self, texto):
        self.mensajes.append(texto)
        del self.mensajes[:-5]

    # ---------- comandos ----------
    def enviar_comandos(self):
        self.leer_teclado()

        msg = Vector3()
        msg.x = float(self.roll_sp)
        msg.y = float(self.pitch_sp)
        msg.z = float(self.yaw_rate_sp)
        self.pub_actitud.publish(msg)

        # Las consignas vuelven solas al centro: sin detección de tecla soltada,
        # mantenerlas fijas sería como dejar el stick trabado.
        self.roll_sp *= DECAIMIENTO
        self.pitch_sp *= DECAIMIENTO
        self.yaw_rate_sp *= DECAIMIENTO
        for nombre in ('roll_sp', 'pitch_sp', 'yaw_rate_sp'):
            if abs(getattr(self, nombre)) < 0.05:
                setattr(self, nombre, 0.0)

    def enviar_altura(self):
        msg = Float32()
        msg.data = float(self.altura_objetivo)
        self.pub_altura.publish(msg)
        self.altura_enviada = True

    def pedir_armado(self, armar):
        """Por servicio si está disponible (así se ve el motivo del rechazo),
        por tópico si no."""
        if self.cli_armar.service_is_ready():
            peticion = Armar.Request()
            peticion.armar = armar
            futuro = self.cli_armar.call_async(peticion)
            futuro.add_done_callback(self.cb_respuesta_armado)
        else:
            self.pub_armar.publish(Bool(data=armar))
            self.aviso('-> %s (por topico)' % ('armar' if armar else 'desarmar'))

    def cb_respuesta_armado(self, futuro):
        try:
            res = futuro.result()
        except Exception as e:                                   # noqa: BLE001
            self.aviso('el servicio de armado fallo: %s' % e)
            return
        if res.aceptado:
            self.aviso('-> armado aceptado')
        else:
            self.aviso('armado RECHAZADO: %s' % res.motivo)

    def leer_teclado(self):
        while select.select([sys.stdin], [], [], 0)[0]:
            tecla = sys.stdin.read(1)
            if not tecla:
                return
            self.procesar_tecla(tecla)

    def procesar_tecla(self, tecla):
        if tecla == 'a':
            if not self.altura_enviada:
                self.aviso('fija primero la altura objetivo (w/s) antes de armar')
            else:
                self.pedir_armado(True)
        elif tecla == 'd':
            self.pedir_armado(False)
        elif tecla == ' ':
            self.pub_emergencia.publish(Bool(data=True))
            self.aviso('-> EMERGENCIA')
        elif tecla == 'w':
            self.altura_objetivo = min(ALTURA_MAX, self.altura_objetivo + PASO_ALTURA)
            self.enviar_altura()
        elif tecla == 's':
            self.altura_objetivo = max(0.1, self.altura_objetivo - PASO_ALTURA)
            self.enviar_altura()
        elif tecla == 'i':
            self.pitch_sp = min(ANGULO_MAX, self.pitch_sp + PASO_ANGULO)
        elif tecla == 'k':
            self.pitch_sp = max(-ANGULO_MAX, self.pitch_sp - PASO_ANGULO)
        elif tecla == 'j':
            self.roll_sp = max(-ANGULO_MAX, self.roll_sp - PASO_ANGULO)
        elif tecla == 'l':
            self.roll_sp = min(ANGULO_MAX, self.roll_sp + PASO_ANGULO)
        elif tecla == 'q':
            self.yaw_rate_sp = max(-YAW_MAX, self.yaw_rate_sp - PASO_YAW)
        elif tecla == 'e':
            self.yaw_rate_sp = min(YAW_MAX, self.yaw_rate_sp + PASO_YAW)
        elif tecla == 'x':
            self.roll_sp = self.pitch_sp = self.yaw_rate_sp = 0.0
            self.aviso('actitud centrada')
        elif tecla == 'g':
            ganancias = [float(v) for v in self.declarar_ganancias()]
            self.pub_ganancias.publish(Float32MultiArray(data=ganancias))
            self.aviso('-> ganancias %s' % ganancias)
        elif tecla == 'h':
            self.aviso('a armar, d desarmar, ESPACIO emergencia, w/s altura, '
                       'i/k/j/l actitud, q/e yaw, x centrar, g ganancias')

    def declarar_ganancias(self):
        # Se leen de parámetros ROS para poder sintonizar sin editar código:
        #   ros2 run robot_bringup estacion_tierra --ros-args -p kp_rate:=0.0025
        nombres = ['kp_ang', 'kp_rate', 'ki_rate', 'kd_rate', 'kp_yaw', 'ki_yaw']
        valores = []
        for nombre, defecto in zip(nombres, GANANCIAS_DEF):
            if not self.has_parameter(nombre):
                self.declare_parameter(nombre, defecto)
            valores.append(self.get_parameter(nombre).value)
        return valores

    # ---------- panel ----------
    def pintar(self):
        roll, pitch, yaw = self.rpy
        modo = MODOS.get(self.estado.modo, '?') if self.estado else 'sin telemetria'
        descripcion = self.estado.descripcion if self.estado else '-'
        a = self.alt
        s = self.sen

        lineas = [
            '\033[H\033[2J' + '=' * 64,
            '  DRON F450 - ESTACION DE TIERRA ROS 2',
            '=' * 64,
            '  MODO: %-12s  %s' % (modo, descripcion),
            '',
            '  Actitud    roll %7.2f   pitch %7.2f   yaw %7.2f  [deg]' % (roll, pitch, yaw),
            '  Consigna   roll %7.2f   pitch %7.2f   yaw %7.2f  [deg, deg/s]'
            % (self.roll_sp, self.pitch_sp, self.yaw_rate_sp),
            '',
        ]

        if a is None:
            lineas += ['  Altura     (sin telemetria)', '']
        else:
            lineas += [
                '  Altura     fusion %6.2f  lidar %6.2f%s  baro %6.2f  [m]'
                % (a.fusionada, a.lidar, ' ' if a.lidar_valido else '!', a.baro),
                '             consigna %6.2f  vz %+6.2f m/s   objetivo %.2f %s'
                % (a.consigna, a.velocidad_vertical, self.altura_objetivo,
                   '' if self.altura_enviada else '<sin enviar>'),
                '',
            ]

        if s is None:
            lineas += ['  Control    (sin telemetria)']
        else:
            lineas += [
                '  Control    throttle %5.3f  vz_sp %+5.2f m/s  a_sp %+5.2f m/s2'
                % (s.throttle, s.velocidad_vertical_deseada,
                   s.aceleracion_vertical_deseada),
                '             corr roll %+6.3f  pitch %+6.3f  yaw %+6.3f'
                % (s.correccion_roll, s.correccion_pitch, s.correccion_yaw),
                '  Motores    %5.0f  %5.0f  %5.0f  %5.0f  [cuentas PWM]'
                % tuple(s.motores),
            ]

        lineas += [
            '',
            '  a armar | d desarmar | ESPACIO emergencia | w/s altura',
            '  i/k pitch | j/l roll | q/e yaw | x centrar | g ganancias',
            '-' * 64,
        ]
        lineas += ['  ' + m for m in self.mensajes]
        sys.stdout.write('\n'.join(lineas) + '\n')
        sys.stdout.flush()


def main():
    rclpy.init()
    nodo = EstacionTierra()

    interactivo = sys.stdin.isatty()
    ajustes = None
    if interactivo:
        ajustes = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
    else:
        nodo.get_logger().warn('stdin no es un terminal: teclado deshabilitado')

    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if ajustes is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, ajustes)
            os.system('stty sane')
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
