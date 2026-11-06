#!/usr/bin/env python3
"""Corredor de escenarios contra una simulación en marcha.

    ros2 run robot_tests corredor --ros-args -p escenario:=<ruta.yaml>
    ros2 run robot_tests corredor --ros-args -p escenario:=hover_1m

Ejecuta una secuencia de acciones con tiempos (fijar altura, armar, parar el
latido...), registra la telemetría y comprueba los criterios del escenario.
Imprime una tabla y sale con código 0 si se cumplen todos, 1 si no: así vale
igual lanzado a mano o desde pytest.

Publica `/dron/cmd_actitud` a 20 Hz mientras el escenario no diga lo contrario,
porque ese flujo es el latido del enlace: si se corta más de TIMEOUT_CMD_S el
dron inicia el aterrizaje por failsafe (y hay un escenario que comprueba
justamente eso).
"""

import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from robot_interfaces.msg import EstadoAltura, EstadoDron, SenalesControl
from geometry_msgs.msg import Vector3, Wrench
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32

FREQ_LATIDO = 20.0
FREQ_MUESTREO = 20.0

NOMBRE_MODO = {0: 'DESARMADO', 1: 'ARMADO', 2: 'ATERRIZAJE', 3: 'DETENIDO'}


def ruta_escenario(nombre):
    if os.path.isfile(nombre):
        return nombre
    base = os.path.join(get_package_share_directory('robot_tests'), 'test_scenarios')
    for candidato in (nombre, nombre + '.yaml'):
        ruta = os.path.join(base, candidato)
        if os.path.isfile(ruta):
            return ruta
    raise FileNotFoundError('no encuentro el escenario %r (busque en %s)' % (nombre, base))


class Corredor(Node):

    def __init__(self):
        super().__init__('corredor_escenarios')

        self.declare_parameter('escenario', 'hover_1m')
        self.declare_parameter('espera_inicial', 3.0)
        self.declare_parameter('espera_telemetria', 30.0)
        escenario = self.get_parameter('escenario').value

        with open(ruta_escenario(escenario)) as f:
            self.esc = yaml.safe_load(f)
        self.pasos = sorted(self.esc.get('pasos', []), key=lambda p: p['t'])
        self.criterios = self.esc.get('criterios', {})
        self.espera_inicial = float(self.get_parameter('espera_inicial').value)

        self.pub_actitud = self.create_publisher(Vector3, 'dron/cmd_actitud',
                                                 qos_profile_sensor_data)
        self.pub_altura = self.create_publisher(Float32, 'dron/cmd_altura', 10)
        self.pub_armar = self.create_publisher(Bool, 'dron/armar', 10)
        self.pub_emergencia = self.create_publisher(Bool, 'dron/emergencia', 10)
        self.pub_perturbacion = self.create_publisher(Wrench, '/sim/perturbacion', 10)

        self.create_subscription(Odometry, '/sim/odom_real', self.cb_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(EstadoAltura, 'dron/estado_altura', self.cb_altura,
                                 qos_profile_sensor_data)
        self.create_subscription(SenalesControl, 'dron/senales_control', self.cb_senales,
                                 qos_profile_sensor_data)
        self.create_subscription(EstadoDron, 'dron/estado_dron', self.cb_estado, 10)

        self.z_real = 0.0
        self.alt = None
        self.sen = None
        self.modo = 0
        self.motivo = 0
        self.latido = True
        self.muestras = []
        self.eventos = []
        self.t0 = None
        self.siguiente_paso = 0
        self.terminado = False
        self.veredicto = None
        self.t_arranque = self.ahora()
        self.limite_telemetria = float(self.get_parameter('espera_telemetria').value)

        self.create_timer(1.0 / FREQ_LATIDO, self.tick_latido)
        self.create_timer(1.0 / FREQ_MUESTREO, self.tick_muestreo)

        print('escenario: %s' % self.esc.get('nombre', escenario))
        print(self.esc.get('descripcion', '').strip())

    # ---------- utilidades ----------
    def ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def transcurrido(self):
        return 0.0 if self.t0 is None else self.ahora() - self.t0

    def anotar(self, texto):
        self.eventos.append((self.transcurrido(), texto))
        print('  [t=%5.1f s] %s' % (self.transcurrido(), texto))

    # ---------- suscripciones ----------
    def cb_odom(self, msg):
        self.z_real = msg.pose.pose.position.z

    def cb_altura(self, msg):
        self.alt = msg

    def cb_senales(self, msg):
        self.sen = msg

    def cb_estado(self, msg):
        if msg.modo != self.modo or msg.motivo_failsafe != self.motivo:
            self.modo, self.motivo = msg.modo, msg.motivo_failsafe
            if self.t0 is not None:
                self.anotar('estado -> %s' % msg.descripcion)

    # ---------- ejecución ----------
    def tick_latido(self):
        if self.latido and not self.terminado:
            self.pub_actitud.publish(Vector3())

    def tick_muestreo(self):
        # Arranca el reloj del escenario cuando hay telemetría: así no se cuentan
        # los segundos de carga de Gazebo.
        if self.t0 is None:
            if self.alt is None:
                # Sin telemetría no hay nada que medir: se falla con un mensaje
                # claro en vez de quedarse colgado para siempre.
                if self.ahora() - self.t_arranque > self.limite_telemetria:
                    print('\nFALLA: no llego telemetria en %.0f s. Esta corriendo la '
                          'simulacion?' % self.limite_telemetria)
                    self.veredicto = False
                    self.terminado = True
                return
            if self.espera_inicial > 0.0:
                self.espera_inicial -= 1.0 / FREQ_MUESTREO
                return
            self.t0 = self.ahora()
            self.anotar('telemetria viva, empieza el escenario')

        t = self.transcurrido()

        while (self.siguiente_paso < len(self.pasos)
               and t >= self.pasos[self.siguiente_paso]['t']):
            self.ejecutar(self.pasos[self.siguiente_paso])
            self.siguiente_paso += 1

        if self.alt is not None:
            self.muestras.append({
                't': t, 'z_real': self.z_real,
                'fusionada': self.alt.fusionada, 'lidar': self.alt.lidar,
                'baro': self.alt.baro, 'consigna': self.alt.consigna,
                'vz': self.alt.velocidad_vertical,
                'throttle': self.sen.throttle if self.sen else 0.0,
                'vz_sp': self.sen.velocidad_vertical_deseada if self.sen else 0.0,
                'acc_sp': self.sen.aceleracion_vertical_deseada if self.sen else 0.0,
                'corr_roll': self.sen.correccion_roll if self.sen else 0.0,
                'modo': self.modo,
            })

        if t >= float(self.esc.get('duracion', 25.0)) and not self.terminado:
            self.terminado = True
            self.evaluar()

    def ejecutar(self, paso):
        accion = paso['accion']
        if accion == 'fijar_altura':
            self.pub_altura.publish(Float32(data=float(paso['valor'])))
            self.anotar('fijar altura = %.2f m' % paso['valor'])
        elif accion == 'armar':
            self.pub_armar.publish(Bool(data=True))
            self.anotar('armar')
        elif accion == 'desarmar':
            self.pub_armar.publish(Bool(data=False))
            self.anotar('desarmar')
        elif accion == 'emergencia':
            self.pub_emergencia.publish(Bool(data=True))
            self.anotar('emergencia')
        elif accion == 'parar_latido':
            self.latido = False
            self.anotar('se corta el latido /dron/cmd_actitud')
        elif accion == 'reanudar_latido':
            self.latido = True
            self.anotar('se reanuda el latido')
        elif accion == 'perturbar':
            w = Wrench()
            w.force.z = float(paso.get('fuerza_z', 0.0))
            w.torque.x = float(paso.get('par_x', 0.0))
            w.torque.y = float(paso.get('par_y', 0.0))
            self.pub_perturbacion.publish(w)
            self.anotar('perturbacion: fuerza_z %.2f N, par %.3f, %.3f N*m'
                        % (w.force.z, w.torque.x, w.torque.y))
        else:
            self.anotar('accion desconocida: %r' % accion)

    # ---------- evaluación ----------
    def ventana(self):
        desde, hasta = self.criterios.get('ventana', [-5.0, 1e9])
        if desde < 0:      # negativo = últimos N segundos
            fin = self.muestras[-1]['t'] if self.muestras else 0.0
            desde, hasta = fin + desde, fin
        return [m for m in self.muestras if desde <= m['t'] <= hasta]

    def evaluar(self):
        print('\n%d muestras en %.1f s' % (len(self.muestras), self.transcurrido()))
        paso = max(1, len(self.muestras) // 18)
        print(' %5s %8s %8s %8s %7s %7s %7s %7s  %s'
              % ('t[s]', 'z_real', 'fusion', 'consig', 'vz', 'vz_sp', 'a_sp',
                 'thr', 'modo'))
        for m in self.muestras[::paso]:
            print(' %5.1f %8.3f %8.3f %8.2f %+7.2f %+7.2f %+7.2f %7.3f  %s'
                  % (m['t'], m['z_real'], m['fusionada'], m['consigna'], m['vz'],
                     m['vz_sp'], m['acc_sp'], m['throttle'],
                     NOMBRE_MODO.get(m['modo'], '?')))

        vent = self.ventana()
        fallos = []
        resumen = []

        if not vent:
            fallos.append('la ventana de evaluacion quedo vacia')
        else:
            zs = [m['z_real'] for m in vent]
            objetivo = float(self.criterios.get('altura_objetivo', 0.0))
            medio = sum(zs) / len(zs)
            resumen.append('z_real medio %.3f m (min %.3f, max %.3f) en %.1f-%.1f s'
                           % (medio, min(zs), max(zs), vent[0]['t'], vent[-1]['t']))
            cs = [m['corr_roll'] for m in vent]
            media_c = sum(cs) / len(cs)
            resumen.append('correccion de roll: media %+.4f, desviacion %.4f'
                           % (media_c, (sum((c - media_c) ** 2 for c in cs) / len(cs)) ** 0.5))

            if 'error_max' in self.criterios:
                error = abs(medio - objetivo)
                limite = float(self.criterios['error_max'])
                resumen.append('error de regimen %.3f m (limite %.3f)' % (error, limite))
                if error > limite:
                    fallos.append('error de regimen %.3f m > %.3f' % (error, limite))

            if 'pico_max' in self.criterios:
                pico = max(m['z_real'] for m in self.muestras)
                limite = float(self.criterios['pico_max'])
                resumen.append('pico %.3f m (limite %.3f)' % (pico, limite))
                if pico > limite:
                    fallos.append('pico %.3f m > %.3f' % (pico, limite))

            if 'desviacion_max' in self.criterios:
                desv = (sum((z - medio) ** 2 for z in zs) / len(zs)) ** 0.5
                limite = float(self.criterios['desviacion_max'])
                resumen.append('desviacion %.3f m (limite %.3f)' % (desv, limite))
                if desv > limite:
                    fallos.append('desviacion %.3f m > %.3f' % (desv, limite))

        if 'modo_final' in self.criterios:
            esperado = int(self.criterios['modo_final'])
            resumen.append('modo final %s (esperado %s)'
                           % (NOMBRE_MODO.get(self.modo), NOMBRE_MODO.get(esperado)))
            if self.modo != esperado:
                fallos.append('modo final %s != %s' % (NOMBRE_MODO.get(self.modo),
                                                       NOMBRE_MODO.get(esperado)))

        if 'motivo_final' in self.criterios:
            esperado = int(self.criterios['motivo_final'])
            resumen.append('motivo final %d (esperado %d)' % (self.motivo, esperado))
            if self.motivo != esperado:
                fallos.append('motivo final %d != %d' % (self.motivo, esperado))

        if 'z_final_max' in self.criterios and self.muestras:
            z_final = self.muestras[-1]['z_real']
            limite = float(self.criterios['z_final_max'])
            resumen.append('z final %.3f m (limite %.3f)' % (z_final, limite))
            if z_final > limite:
                fallos.append('z final %.3f m > %.3f' % (z_final, limite))

        print('')
        for linea in resumen:
            print('  ' + linea)
        self.veredicto = not fallos
        if fallos:
            print('\nFALLA: ' + '; '.join(fallos))
        else:
            print('\nOK: se cumplen todos los criterios')


def main():
    rclpy.init()
    nodo = Corredor()
    codigo = 1
    try:
        while rclpy.ok() and not nodo.terminado:
            rclpy.spin_once(nodo, timeout_sec=0.1)
        codigo = 0 if nodo.veredicto else 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Deja el dron desarmado: el siguiente escenario arranca limpio. Solo si
        # el contexto sigue vivo: si nos mataron desde fuera ya no se puede
        # publicar, y hacerlo revienta con "publisher's context is invalid".
        if rclpy.ok():
            try:
                nodo.pub_armar.publish(Bool(data=False))
                rclpy.spin_once(nodo, timeout_sec=0.2)
            except Exception:                                    # noqa: BLE001
                pass
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(codigo)


if __name__ == '__main__':
    main()
