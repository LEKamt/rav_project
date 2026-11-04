#!/usr/bin/env python3
"""Controlador de vuelo simulado: hace en Gazebo el papel del ESP32-S3.

Habla exactamente los mismos tópicos que el firmware (`/dron/...`), así que la
misma `estacion_tierra` sirve para la simulación y para el dron real sin cambiar
nada. Lo que cambia es de dónde vienen los sensores y a dónde van los mandos:

    /sim/imu        (250 Hz) -> filtro complementario + cascada de actitud
    /sim/lidar      ( 50 Hz) -> LiDAR
    /sim/odom_real  ( 50 Hz) -> base del barómetro simulado (con deriva y ruido)
    /sim/wrench              <- wrench de cuerpo equivalente a los 4 empujes

El lazo de actitud corre en el callback de la IMU (250 Hz, como la tarea del core
1 del firmware) y el de altura en un temporizador de 50 Hz (como la del core 0).

Publica la telemetría dos veces: en los Float32MultiArray que publica el firmware
(paridad, para que rqt_plot y los scripts valgan igual en sim y en hardware) y en
los mensajes de dron_interfaces, que son los que consumen los nodos del PC.
"""

import math
import random

import rclpy
from dron_control.control import (ControlActitud, ControlAltura, ParametrosControl,
                                  acotar, mezclar)
from dron_interfaces.msg import EstadoAltura, EstadoDron, SenalesControl
from dron_interfaces.srv import Armar
from dron_localization.estimacion import (EstimadorActitud, EstimadorAltura,
                                          ParametrosEstimacion)
from geometry_msgs.msg import Vector3, Wrench
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32

# Modos y motivos: mismos códigos que el firmware y que EstadoDron.msg
DESARMADO, ARMADO, ATERRIZAJE, DETENIDO = range(4)
FS_NINGUNO, FS_BOTON, FS_TIMEOUT_CMD, FS_AGENTE, FS_ROS, FS_IMU = range(6)

NOMBRE_MODO = {DESARMADO: 'DESARMADO', ARMADO: 'ARMADO',
               ATERRIZAJE: 'ATERRIZAJE', DETENIDO: 'DETENIDO'}
NOMBRE_MOTIVO = {FS_NINGUNO: '', FS_BOTON: 'boton fisico',
                 FS_TIMEOUT_CMD: 'timeout de comandos', FS_AGENTE: 'agente perdido',
                 FS_ROS: 'orden por ROS', FS_IMU: 'IMU sin lecturas'}

RAD_A_DEG = 180.0 / math.pi
G = 9.80665

# Cadencias del firmware que hay que imitar (src/config.h)
FREQ_CONTROL = 250.0     # Hz, tarea del core 1
PUB_IMU_HZ = 20.0        # Hz, lo que publica el firmware en /dron/imu

# Ganancias ajustables por parámetro ROS (config/ganancias.yaml)
NOMBRES_GANANCIAS = ('kp_ang', 'kp_rate', 'ki_rate', 'kd_rate', 'kp_yaw', 'ki_yaw',
                     'kp_pos_z', 'kd_pos_z', 'kp_vel_z', 'ki_vel_z', 'kd_vel_z')


class ControladorSim(Node):

    def __init__(self):
        super().__init__('controlador_sim')

        # ---------- parámetros del modelo ----------
        self.declare_parameter('empuje_max_motor', 8.3)      # N por motor (A2212 + 1045 a 3S)
        self.declare_parameter('exponente_empuje', 1.0)      # 1.0 = empuje lineal con el mando
        self.declare_parameter('tau_motor', 0.08)            # s, retardo del conjunto ESC+motor
        self.declare_parameter('brazo_efectivo', 0.1589)     # m, F450 medido en la malla
        self.declare_parameter('k_arrastre_yaw', 0.016)      # m, par de reacción / empuje
        self.declare_parameter('deriva_baro', 0.02)          # m/s de paseo aleatorio
        self.declare_parameter('ruido_baro', 0.08)           # m, desviación típica

        self.empuje_max = self.get_parameter('empuje_max_motor').value
        self.exponente = self.get_parameter('exponente_empuje').value
        self.tau_motor = self.get_parameter('tau_motor').value
        self.brazo = self.get_parameter('brazo_efectivo').value
        self.k_yaw = self.get_parameter('k_arrastre_yaw').value
        self.deriva_baro = self.get_parameter('deriva_baro').value
        self.ruido_baro = self.get_parameter('ruido_baro').value

        # ---------- control y estimación ----------
        self.par = ParametrosControl()
        self.par_est = ParametrosEstimacion()
        self.est_actitud = EstimadorActitud(self.par_est)
        self.actitud = ControlActitud(self.par)
        self.est_altura = EstimadorAltura(self.par_est)
        self.altura = ControlAltura(self.par)
        self.cargar_ganancias()

        self.modo = DESARMADO
        self.motivo = FS_NINGUNO
        self.setpoint_manual = False
        self.empujes = [0.0] * 4          # N, con retardo de motor
        self.mandos = [0.0] * 4           # normalizados
        self.throttle_aplicado = 0.0
        self.t_rampa = None
        self.objetivo_descenso = 0.0
        self.t_descenso_cero = None

        self.t_ultimo_cmd = self.ahora()
        self.t_ultima_imu = None
        self.t_imu_prev = None
        self.dt_acumulado = 0.0
        self.t_pub_imu = 0.0

        # Barómetro simulado
        self.z_real = 0.0
        self.offset_baro = 0.0
        self.paseo_baro = 0.0
        self.contador_baro = 0
        self.baro_referenciado = False

        # ---------- interfaz con Gazebo ----------
        self.pub_wrench = self.create_publisher(Wrench, '/sim/wrench', 10)
        self.create_subscription(Imu, '/sim/imu', self.cb_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/sim/lidar', self.cb_lidar,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/sim/odom_real', self.cb_odom,
                                 qos_profile_sensor_data)

        # ---------- interfaz igual que el firmware ----------
        self.pub_imu = self.create_publisher(Imu, 'dron/imu', qos_profile_sensor_data)
        self.pub_alturas = self.create_publisher(Float32MultiArray, 'dron/alturas',
                                                 qos_profile_sensor_data)
        self.pub_control = self.create_publisher(Float32MultiArray, 'dron/control',
                                                 qos_profile_sensor_data)
        self.pub_motores = self.create_publisher(Float32MultiArray, 'dron/motores',
                                                 qos_profile_sensor_data)
        self.pub_estado = self.create_publisher(Int32, 'dron/estado', 10)

        # ---------- interfaz con mensajes propios ----------
        self.pub_estado_dron = self.create_publisher(EstadoDron, 'dron/estado_dron', 10)
        self.pub_estado_altura = self.create_publisher(EstadoAltura, 'dron/estado_altura',
                                                       qos_profile_sensor_data)
        self.pub_senales = self.create_publisher(SenalesControl, 'dron/senales_control',
                                                 qos_profile_sensor_data)
        self.srv_armar = self.create_service(Armar, 'dron/armar_srv', self.cb_srv_armar)

        self.create_subscription(Vector3, 'dron/cmd_actitud', self.cb_cmd_actitud,
                                 qos_profile_sensor_data)
        self.create_subscription(Float32, 'dron/cmd_altura', self.cb_cmd_altura, 10)
        self.create_subscription(Bool, 'dron/armar', self.cb_armar, 10)
        self.create_subscription(Bool, 'dron/emergencia', self.cb_emergencia, 10)
        self.create_subscription(Float32MultiArray, 'dron/cmd_ganancias',
                                 self.cb_ganancias, 10)

        self.create_timer(1.0 / 50.0, self.lazo_lento)
        self.create_timer(1.0 / 5.0, self.publicar_lento)

        self.get_logger().info(
            'controlador_sim listo: empuje max %.1f N/motor, hover %.2f, '
            'ganancias altura pos(%.2f, %.2f) vel(%.2f, %.2f, %.2f)'
            % (self.empuje_max, self.par.THROTTLE_HOVER,
               self.altura.pos_z.kp, self.altura.pos_z.kd,
               self.altura.vel_z.kp, self.altura.vel_z.ki, self.altura.vel_z.kd))

    # ===================== parámetros =====================
    def cargar_ganancias(self):
        """Las ganancias vienen de config/ganancias.yaml, no del código: así se
        sintoniza sin reinstalar el paquete. Los valores por defecto son los de
        las clases, que son los del firmware."""
        defectos = {
            'kp_ang': self.actitud.kp_ang, 'kp_rate': self.actitud.kp_rate,
            'ki_rate': self.actitud.ki_rate, 'kd_rate': self.actitud.kd_rate,
            'kp_yaw': self.actitud.kp_yaw, 'ki_yaw': self.actitud.ki_yaw,
            'kp_pos_z': self.altura.pos_z.kp, 'kd_pos_z': self.altura.pos_z.kd,
            'kp_vel_z': self.altura.vel_z.kp, 'ki_vel_z': self.altura.vel_z.ki,
            'kd_vel_z': self.altura.vel_z.kd,
        }
        valores = {}
        for nombre in NOMBRES_GANANCIAS:
            self.declare_parameter(nombre, float(defectos[nombre]))
            valores[nombre] = float(self.get_parameter(nombre).value)

        self.actitud.set_ganancias([valores['kp_ang'], valores['kp_rate'],
                                    valores['ki_rate'], valores['kd_rate'],
                                    valores['kp_yaw'], valores['ki_yaw']])
        self.altura.set_ganancias_altura(valores['kp_pos_z'], valores['kd_pos_z'],
                                         valores['kp_vel_z'], valores['ki_vel_z'],
                                         valores['kd_vel_z'])

    # ===================== utilidades =====================
    def ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def descripcion(self):
        texto = NOMBRE_MODO.get(self.modo, '?')
        if self.motivo:
            texto += ' por ' + NOMBRE_MOTIVO.get(self.motivo, '?')
        return texto

    def armar(self):
        """Devuelve (aceptado, motivo del rechazo)."""
        if self.modo not in (DESARMADO, DETENIDO):
            return False, 'el dron no esta desarmado'
        if not self.setpoint_manual:
            return False, 'falta fijar la altura objetivo'
        self.actitud.reset(self.est_actitud.vel_roll, self.est_actitud.vel_pitch)
        self.altura.reset(self.est_altura.altura)
        self.motivo = FS_NINGUNO
        self.t_ultimo_cmd = self.ahora()
        self.t_rampa = self.ahora()
        self.modo = ARMADO
        self.get_logger().info('ARMADO')
        return True, ''

    def desarmar(self):
        self.modo = DESARMADO
        self.setpoint_manual = False
        self.motivo = FS_NINGUNO
        self.t_rampa = None
        self.get_logger().info('DESARMADO')

    def pedir_aterrizaje(self, motivo):
        # El descenso es nivelado: si el enlace cayó con una consigna de
        # inclinación activa, mantenerla sacaría al dron de lado.
        self.actitud.sp_roll = 0.0
        self.actitud.sp_pitch = 0.0
        self.actitud.sp_yaw_rate = 0.0
        if self.modo == ARMADO:
            self.motivo = motivo
            self.modo = ATERRIZAJE
            self.objetivo_descenso = self.est_altura.altura
            self.t_descenso_cero = None
            self.get_logger().warn('ATERRIZAJE de emergencia: %s'
                                   % NOMBRE_MOTIVO.get(motivo, '?'))
        elif self.modo != ATERRIZAJE:
            self.modo = DETENIDO
            self.motivo = motivo

    # ===================== callbacks de mando =====================
    def cb_cmd_actitud(self, msg):
        self.t_ultimo_cmd = self.ahora()
        if self.modo == ATERRIZAJE:
            return      # el descenso es nivelado
        self.actitud.sp_roll = acotar(msg.x, -self.par.ANG_SP_MAX, self.par.ANG_SP_MAX)
        self.actitud.sp_pitch = acotar(msg.y, -self.par.ANG_SP_MAX, self.par.ANG_SP_MAX)
        self.actitud.sp_yaw_rate = acotar(msg.z, -self.par.YAW_RATE_SP_MAX,
                                          self.par.YAW_RATE_SP_MAX)

    def cb_cmd_altura(self, msg):
        self.t_ultimo_cmd = self.ahora()
        if self.modo == ATERRIZAJE:
            return
        if 0.05 < msg.data <= self.par.ALTURA_SP_MAX:
            self.altura.objetivo = msg.data
            self.setpoint_manual = True

    def cb_armar(self, msg):
        self.t_ultimo_cmd = self.ahora()
        if msg.data:
            aceptado, motivo = self.armar()
            if not aceptado:
                self.get_logger().warn('armado rechazado: %s' % motivo)
        else:
            self.desarmar()

    def cb_srv_armar(self, peticion, respuesta):
        self.t_ultimo_cmd = self.ahora()
        if peticion.armar:
            respuesta.aceptado, respuesta.motivo = self.armar()
        else:
            self.desarmar()
            respuesta.aceptado, respuesta.motivo = True, ''
        return respuesta

    def cb_emergencia(self, msg):
        self.t_ultimo_cmd = self.ahora()
        if msg.data:
            self.pedir_aterrizaje(FS_ROS)

    def cb_ganancias(self, msg):
        self.t_ultimo_cmd = self.ahora()
        if self.modo != DESARMADO:
            self.get_logger().warn('ganancias ignoradas: el dron no esta desarmado')
            return
        if len(msg.data) >= 6:
            self.actitud.set_ganancias(list(msg.data))
            self.get_logger().info('ganancias de actitud = %s' % self.actitud.ganancias())

    # ===================== sensores =====================
    def cb_lidar(self, msg):
        # Un solo haz. Sin eco el plugin devuelve FLT_MAX: eso es "fuera de
        # alcance", igual que los 8190 mm que devuelve el VL53L0X real.
        if not msg.ranges:
            return
        d = float(msg.ranges[0])
        valido = math.isfinite(d) and msg.range_min < d < msg.range_max
        self.est_altura.set_lidar(d, valido)

    def cb_odom(self, msg):
        self.z_real = msg.pose.pose.position.z
        if not self.baro_referenciado:
            # El firmware promedia 50 muestras en el arranque: el cero del
            # barómetro es la altura inicial, no el nivel del mar.
            self.offset_baro = self.z_real
            self.baro_referenciado = True

        # Barómetro simulado a 10 Hz con paseo aleatorio + ruido
        self.contador_baro += 1
        if self.contador_baro >= 5:
            self.contador_baro = 0
            self.paseo_baro += random.gauss(0.0, self.deriva_baro * 0.1)
            self.paseo_baro = acotar(self.paseo_baro, -0.5, 0.5)
            medida = (self.z_real - self.offset_baro) + self.paseo_baro \
                + random.gauss(0.0, self.ruido_baro)
            self.est_altura.set_baro(medida)

    # ===================== lazo rápido (250 Hz, callback de la IMU) =====================
    def cb_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.t_ultima_imu = self.ahora()

        dt = 1.0 / FREQ_CONTROL if self.t_imu_prev is None else t - self.t_imu_prev
        self.t_imu_prev = t
        if dt <= 0.0 or dt > 0.1:
            return

        # El plugin de IMU de Gazebo publica en cada paso de física (~500 Hz)
        # aunque el <sensor> pida 250, así que se acumula el tiempo y el lazo se
        # ejecuta a FREQ_CONTROL: el firmware corre a 250 Hz y la simulación solo
        # vale como banco de pruebas si ve la misma cadencia.
        self.dt_acumulado += dt
        if self.dt_acumulado < 1.0 / FREQ_CONTROL - 1e-9:
            return
        dt = self.dt_acumulado
        self.dt_acumulado = 0.0

        acc = (msg.linear_acceleration.x / G,
               msg.linear_acceleration.y / G,
               msg.linear_acceleration.z / G)
        gyr = (msg.angular_velocity.x * RAD_A_DEG,
               msg.angular_velocity.y * RAD_A_DEG,
               msg.angular_velocity.z * RAD_A_DEG)

        self.est_actitud.actualizar(acc, gyr, dt)

        if self.modo in (ARMADO, ATERRIZAJE):
            self.actitud.actualizar(self.est_actitud, dt)
            throttle = self.altura.throttle_base

            # Rampa de arranque, igual que en motores.cpp
            if self.t_rampa is not None:
                transcurrido = self.ahora() - self.t_rampa
                if transcurrido >= self.par.RAMP_TIME_S:
                    self.t_rampa = None
                else:
                    throttle *= transcurrido / self.par.RAMP_TIME_S

            self.mandos = mezclar(throttle, self.actitud.corr_roll,
                                  self.actitud.corr_pitch, self.actitud.corr_yaw,
                                  self.par)
            self.throttle_aplicado = sum(self.mandos) / 4.0
        else:
            self.actitud.reset(self.est_actitud.vel_roll, self.est_actitud.vel_pitch)
            self.mandos = [0.0] * 4
            self.throttle_aplicado = 0.0

        self.aplicar_empujes(dt)
        self.publicar_imu(msg)

    def aplicar_empujes(self, dt):
        """Modelo de motor + wrench de cuerpo equivalente a los 4 empujes."""
        alpha = dt / (dt + self.tau_motor)
        for i, mando in enumerate(self.mandos):
            objetivo = self.empuje_max * (max(mando, 0.0) ** self.exponente)
            self.empujes[i] += alpha * (objetivo - self.empujes[i])

        t1, t2, t3, t4 = self.empujes
        w = Wrench()
        w.force.z = t1 + t2 + t3 + t4
        # r x F con los motores en (+-brazo, +-brazo): tau_x = sum(y_i * T_i)
        w.torque.x = self.brazo * ((t1 + t4) - (t2 + t3))
        # tau_y = -sum(x_i * T_i): positivo = morro abajo
        w.torque.y = self.brazo * ((t3 + t4) - (t1 + t2))
        # Par de reacción: M2 y M4 giran al revés que M1 y M3
        w.torque.z = self.k_yaw * ((t2 + t4) - (t1 + t3))
        self.pub_wrench.publish(w)

    # ===================== lazo lento (50 Hz) =====================
    def lazo_lento(self):
        dt = 1.0 / 50.0
        self.est_altura.actualizar(dt, self.est_actitud.acc_vertical)

        if self.modo == ATERRIZAJE:
            self.ejecutar_aterrizaje(dt)

        if self.modo in (ARMADO, ATERRIZAJE):
            self.altura.actualizar(self.est_altura.altura, self.est_altura.vz,
                                   self.est_actitud.acc_vertical, dt)

        self.verificar_failsafe()
        self.publicar_rapido()

    def ejecutar_aterrizaje(self, dt):
        if self.est_altura.lidar_valido and \
                self.est_altura.lidar < self.par.ALTURA_CORTE_MOTORES:
            self.get_logger().warn('suelo detectado a %.2f m: motores cortados'
                                   % self.est_altura.lidar)
            self.modo = DETENIDO
            self.setpoint_manual = False
            return

        self.objetivo_descenso = max(
            0.0, self.objetivo_descenso - self.par.VELOCIDAD_DESCENSO * dt)
        self.altura.objetivo = self.objetivo_descenso

        if self.objetivo_descenso <= 0.0:
            if self.t_descenso_cero is None:
                self.t_descenso_cero = self.ahora()
            elif self.ahora() - self.t_descenso_cero > 2.0:
                self.get_logger().warn('objetivo en 0 sin suelo: motores cortados')
                self.modo = DETENIDO
                self.setpoint_manual = False
        else:
            self.t_descenso_cero = None

    def verificar_failsafe(self):
        if self.modo != ARMADO:
            return
        ahora = self.ahora()
        if self.t_ultima_imu is not None and \
                ahora - self.t_ultima_imu > self.par.TIMEOUT_IMU_S:
            self.pedir_aterrizaje(FS_IMU)
            return
        if ahora - self.t_ultimo_cmd > self.par.TIMEOUT_CMD_S:
            self.pedir_aterrizaje(FS_TIMEOUT_CMD)

    # ===================== telemetría =====================
    def publicar_imu(self, msg_sim):
        # Al ritmo del firmware, no al del plugin de Gazebo
        ahora = self.ahora()
        if ahora - self.t_pub_imu < 1.0 / PUB_IMU_HZ:
            return
        self.t_pub_imu = ahora

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Cuaternión del filtro complementario, no la verdad de Gazebo
        x, y, z, w = self.est_actitud.cuaternion()
        msg.orientation.x, msg.orientation.y = x, y
        msg.orientation.z, msg.orientation.w = z, w

        msg.angular_velocity.x = math.radians(self.est_actitud.vel_roll)
        msg.angular_velocity.y = math.radians(self.est_actitud.vel_pitch)
        msg.angular_velocity.z = math.radians(self.est_actitud.vel_yaw)
        msg.linear_acceleration = msg_sim.linear_acceleration
        self.pub_imu.publish(msg)

    def pwm_actual(self):
        span = self.par.PWM_MAX - self.par.PWM_MIN
        if self.modo in (ARMADO, ATERRIZAJE):
            return [float(self.par.PWM_MIN + m * span) for m in self.mandos]
        return [float(self.par.PWM_MIN)] * 4

    def publicar_rapido(self):
        ahora = self.get_clock().now().to_msg()
        lidar = self.est_altura.lidar if self.est_altura.lidar_valido else 0.0
        objetivo = self.altura.objetivo if self.setpoint_manual else 0.0

        # --- paridad con el firmware ---
        self.pub_alturas.publish(Float32MultiArray(data=[
            float(self.est_altura.altura), float(lidar), float(self.est_altura.baro),
            float(self.altura.pos_z.consigna), float(self.est_altura.vz),
            float(objetivo),
        ]))
        self.pub_control.publish(Float32MultiArray(data=[
            float(self.throttle_aplicado),
            float(self.actitud.corr_roll), float(self.actitud.corr_pitch),
            float(self.actitud.corr_yaw),
            float(self.altura.pos_z.salida),    # velocidad vertical deseada [m/s]
            float(self.altura.vel_z.salida),    # aceleración vertical deseada [m/s^2]
            float(self.actitud.rate_sp_roll), float(self.actitud.rate_sp_pitch),
        ]))

        # --- mensajes propios ---
        alt = EstadoAltura()
        alt.header.stamp = ahora
        alt.header.frame_id = 'base_link'
        alt.fusionada = float(self.est_altura.altura)
        alt.lidar = float(lidar)
        alt.baro = float(self.est_altura.baro)
        alt.consigna = float(self.altura.pos_z.consigna)
        alt.velocidad_vertical = float(self.est_altura.vz)
        alt.objetivo = float(objetivo)
        alt.lidar_valido = bool(self.est_altura.lidar_valido)
        self.pub_estado_altura.publish(alt)

        sen = SenalesControl()
        sen.header.stamp = ahora
        sen.header.frame_id = 'base_link'
        sen.throttle = float(self.throttle_aplicado)
        sen.correccion_roll = float(self.actitud.corr_roll)
        sen.correccion_pitch = float(self.actitud.corr_pitch)
        sen.correccion_yaw = float(self.actitud.corr_yaw)
        sen.velocidad_vertical_deseada = float(self.altura.pos_z.salida)
        sen.aceleracion_vertical_deseada = float(self.altura.vel_z.salida)
        sen.rate_deseado_roll = float(self.actitud.rate_sp_roll)
        sen.rate_deseado_pitch = float(self.actitud.rate_sp_pitch)
        sen.motores = self.pwm_actual()
        self.pub_senales.publish(sen)

    def publicar_lento(self):
        self.pub_motores.publish(Float32MultiArray(data=self.pwm_actual()))
        self.pub_estado.publish(Int32(data=int(self.modo + 10 * self.motivo)))

        est = EstadoDron()
        est.header.stamp = self.get_clock().now().to_msg()
        est.modo = int(self.modo)
        est.motivo_failsafe = int(self.motivo)
        est.setpoint_fijado = bool(self.setpoint_manual)
        est.descripcion = self.descripcion()
        self.pub_estado_dron.publish(est)

    def parar_motores(self):
        """El plugin de fuerzas reaplica el último wrench para siempre: hay que
        dejarlo en cero al salir o el dron seguiría empujando sin controlador."""
        self.pub_wrench.publish(Wrench())


def main():
    rclpy.init()
    nodo = ControladorSim()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.parar_motores()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
