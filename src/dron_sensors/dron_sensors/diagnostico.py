#!/usr/bin/env python3
"""Diagnóstico de los sensores del dron.

Vigila lo que el control da por supuesto y publica
diagnostic_msgs/DiagnosticArray en /diagnostics, visible con
`ros2 run rqt_robot_monitor rqt_robot_monitor` o con
`ros2 topic echo /diagnostics`.

Comprueba cuatro cosas, que son las que han fallado de verdad en banco:

  1. Cadencia de la IMU. El lazo de actitud supone 250 Hz; si el bus I2C se
     atasca, el firmware propaga el estado con la última velocidad angular y
     acaba disparando el failsafe de IMU.
  2. Cadencia de la telemetría de altura.
  3. Validez del LiDAR. El VL53L0X no ve superficies negras, agua ni con sol
     directo, y pierde alcance más allá de ~2 m.
  4. Coherencia LiDAR vs barómetro. Si discrepan mucho mientras los dos son
     válidos, el anclaje del barómetro está mal o uno de los dos miente.

No corta el vuelo: eso es trabajo del failsafe del firmware, que no depende del
PC. Esto es para que el operador vea *por qué* algo va mal.
"""

import math
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from dron_interfaces.msg import EstadoAltura, EstadoDron
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

OK = DiagnosticStatus.OK
AVISO = DiagnosticStatus.WARN
ERROR = DiagnosticStatus.ERROR

G = 9.80665


class Tasa:
    """Frecuencia media de un tópico en una ventana deslizante."""

    def __init__(self, ventana=50):
        self.marcas = deque(maxlen=ventana)

    def marcar(self, t):
        self.marcas.append(t)

    def hz(self):
        if len(self.marcas) < 2:
            return 0.0
        span = self.marcas[-1] - self.marcas[0]
        return (len(self.marcas) - 1) / span if span > 0 else 0.0

    def edad(self, ahora):
        return float('inf') if not self.marcas else ahora - self.marcas[-1]


class Diagnostico(Node):

    def __init__(self):
        super().__init__('diagnostico_sensores')

        self.declare_parameter('hz_imu_esperada', 20.0)      # lo que publica el firmware
        self.declare_parameter('hz_imu_minima', 10.0)
        self.declare_parameter('hz_altura_esperada', 50.0)
        self.declare_parameter('hz_altura_minima', 20.0)
        self.declare_parameter('silencio_max', 1.0)          # s sin mensajes -> ERROR
        self.declare_parameter('discrepancia_max', 0.30)     # m entre lidar y baro
        self.declare_parameter('acc_reposo_max', 0.15)       # g de desvio sobre 1 g
        self.declare_parameter('frecuencia', 2.0)

        self.p = {n: self.get_parameter(n).value for n in (
            'hz_imu_esperada', 'hz_imu_minima', 'hz_altura_esperada',
            'hz_altura_minima', 'silencio_max', 'discrepancia_max',
            'acc_reposo_max')}

        self.tasa_imu = Tasa()
        self.tasa_altura = Tasa()
        self.imu = None
        self.alt = None
        self.estado = None
        self.muestras_lidar = deque(maxlen=100)

        self.create_subscription(Imu, 'dron/imu', self.cb_imu, qos_profile_sensor_data)
        self.create_subscription(EstadoAltura, 'dron/estado_altura', self.cb_altura,
                                 qos_profile_sensor_data)
        self.create_subscription(EstadoDron, 'dron/estado_dron', self.cb_estado, 10)

        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0 / self.get_parameter('frecuencia').value, self.publicar)

    def ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_imu(self, msg):
        self.imu = msg
        self.tasa_imu.marcar(self.ahora())

    def cb_altura(self, msg):
        self.alt = msg
        self.tasa_altura.marcar(self.ahora())
        self.muestras_lidar.append(bool(msg.lidar_valido))

    def cb_estado(self, msg):
        self.estado = msg

    # ---------- comprobaciones ----------
    def estado_cadencia(self, nombre, tasa, esperada, minima, ahora):
        st = DiagnosticStatus()
        st.name = nombre
        st.hardware_id = 'dron_f450'
        hz = tasa.hz()
        edad = tasa.edad(ahora)
        st.values = [KeyValue(key='hz', value='%.1f' % hz),
                     KeyValue(key='esperada_hz', value='%.1f' % esperada),
                     KeyValue(key='ultimo_mensaje_s', value='%.2f' % edad)]
        if edad > self.p['silencio_max']:
            st.level, st.message = ERROR, 'sin mensajes desde hace %.1f s' % edad
        elif hz < minima:
            st.level, st.message = AVISO, 'cadencia baja: %.1f Hz' % hz
        else:
            st.level, st.message = OK, '%.1f Hz' % hz
        return st

    def estado_lidar(self):
        st = DiagnosticStatus()
        st.name = 'LiDAR VL53L0X'
        st.hardware_id = 'dron_f450'
        if self.alt is None:
            st.level, st.message = ERROR, 'sin telemetria de altura'
            return st

        validas = sum(self.muestras_lidar)
        total = max(1, len(self.muestras_lidar))
        porcentaje = 100.0 * validas / total
        st.values = [
            KeyValue(key='valido', value=str(self.alt.lidar_valido)),
            KeyValue(key='medida_m', value='%.3f' % self.alt.lidar),
            KeyValue(key='validas_pct', value='%.0f' % porcentaje),
        ]
        if porcentaje < 20.0:
            # Normal por encima de 2 m: ahí manda el barómetro
            st.level = AVISO
            st.message = 'el LiDAR casi no mide (%.0f%% validas)' % porcentaje
        elif not self.alt.lidar_valido:
            st.level, st.message = AVISO, 'medida actual no valida (fuera de alcance)'
        else:
            st.level, st.message = OK, '%.2f m' % self.alt.lidar
        return st

    def estado_coherencia(self):
        st = DiagnosticStatus()
        st.name = 'Coherencia LiDAR / barometro'
        st.hardware_id = 'dron_f450'
        if self.alt is None:
            st.level, st.message = ERROR, 'sin telemetria de altura'
            return st
        if not self.alt.lidar_valido:
            st.level, st.message = OK, 'sin LiDAR valido: no se puede comparar'
            return st

        diferencia = abs(self.alt.lidar - self.alt.baro)
        st.values = [KeyValue(key='lidar_m', value='%.3f' % self.alt.lidar),
                     KeyValue(key='baro_m', value='%.3f' % self.alt.baro),
                     KeyValue(key='diferencia_m', value='%.3f' % diferencia)]
        if diferencia > self.p['discrepancia_max']:
            st.level = AVISO
            st.message = ('LiDAR y barometro discrepan %.2f m: revisa el anclaje'
                          % diferencia)
        else:
            st.level, st.message = OK, 'coinciden en %.2f m' % diferencia
        return st

    def estado_imu(self):
        st = DiagnosticStatus()
        st.name = 'IMU MPU-6500'
        st.hardware_id = 'dron_f450'
        if self.imu is None:
            st.level, st.message = ERROR, 'sin datos de IMU'
            return st

        a = self.imu.linear_acceleration
        modulo = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z) / G
        w = self.imu.angular_velocity
        giro = math.degrees(math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z))
        st.values = [KeyValue(key='modulo_acc_g', value='%.3f' % modulo),
                     KeyValue(key='giro_total_dps', value='%.1f' % giro)]

        desarmado = self.estado is not None and self.estado.modo == 0
        if desarmado and abs(modulo - 1.0) > self.p['acc_reposo_max']:
            # En reposo el acelerómetro debe medir 1 g: si no, está mal calibrado
            # o el dron no estaba quieto al arrancar (autoOffsets del firmware).
            st.level = AVISO
            st.message = ('en reposo mide %.2f g: revisa la calibracion (el dron '
                          'debe estar quieto y nivelado al encender)' % modulo)
        elif modulo > 4.0:
            st.level, st.message = AVISO, 'aceleracion de %.1f g: golpe o vibracion' % modulo
        else:
            st.level, st.message = OK, '%.2f g, %.0f deg/s' % (modulo, giro)
        return st

    def publicar(self):
        ahora = self.ahora()
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [
            self.estado_cadencia('Cadencia /dron/imu', self.tasa_imu,
                                 self.p['hz_imu_esperada'], self.p['hz_imu_minima'],
                                 ahora),
            self.estado_cadencia('Cadencia /dron/estado_altura', self.tasa_altura,
                                 self.p['hz_altura_esperada'],
                                 self.p['hz_altura_minima'], ahora),
            self.estado_imu(),
            self.estado_lidar(),
            self.estado_coherencia(),
        ]
        self.pub.publish(msg)


def main():
    rclpy.init()
    nodo = Diagnostico()
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
