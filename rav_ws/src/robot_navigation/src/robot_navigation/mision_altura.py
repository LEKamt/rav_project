#!/usr/bin/env python3
"""Misiones de altura: servidor de acción IrAAltura y ejecutor de perfiles.

Este dron no navega en horizontal (no tiene estimación de posición), así que
"navegación" aquí es el perfil de altura: subir a una cota, mantenerla, pasar a
otra y aterrizar. Es lo que hace repetibles las pruebas de sintonización.

Dos formas de usarlo:

    # como acción, desde otro nodo o a mano
    ros2 action send_goal /ir_a_altura robot_interfaces/action/IrAAltura \\
        "{altura: 1.2, tolerancia: 0.1, tiempo_limite: 25.0}"

    # como perfil completo, sin tocar nada más
    ros2 launch robot_navigation mision.launch.py perfil:=perfil_hover

Publica el latido /dron/cmd_actitud a 20 Hz mientras vive: sin ese flujo el
firmware aterriza por failsafe a los TIMEOUT_CMD_MS. Si la estación de tierra ya
está publicando, los dos flujos conviven (ambos mandan ceros de actitud), pero
conviene no mandar consignas de inclinación desde los dos sitios a la vez.
"""

import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from robot_interfaces.action import IrAAltura
from robot_interfaces.msg import EstadoAltura, EstadoDron
from robot_interfaces.srv import Armar
from geometry_msgs.msg import Vector3
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32

FREQ_LATIDO = 20.0
TOLERANCIA_DEF = 0.10
LIMITE_DEF = 30.0
MUESTRAS_ESTABLE = 10       # ciclos seguidos dentro de tolerancia (1 s a 10 Hz)

DESARMADO, ARMADO, ATERRIZAJE, DETENIDO = range(4)


def ruta_perfil(nombre):
    if os.path.isfile(nombre):
        return nombre
    base = os.path.join(get_package_share_directory('robot_navigation'), 'param')
    for candidato in (nombre, nombre + '.yaml'):
        ruta = os.path.join(base, candidato)
        if os.path.isfile(ruta):
            return ruta
    raise FileNotFoundError('no encuentro el perfil %r (busque en %s)' % (nombre, base))


class MisionAltura(Node):

    def __init__(self):
        super().__init__('mision_altura')

        self.declare_parameter('perfil', '')
        self.declare_parameter('publicar_latido', True)
        self.declare_parameter('espera_telemetria', 10.0)
        perfil = self.get_parameter('perfil').value
        self.publicar_latido = self.get_parameter('publicar_latido').value

        grupo_accion = ReentrantCallbackGroup()
        grupo_datos = ReentrantCallbackGroup()
        grupo_perfil = MutuallyExclusiveCallbackGroup()

        self.pub_altura = self.create_publisher(Float32, 'dron/cmd_altura', 10)
        self.pub_actitud = self.create_publisher(Vector3, 'dron/cmd_actitud',
                                                 qos_profile_sensor_data)
        self.pub_emergencia = self.create_publisher(Bool, 'dron/emergencia', 10)
        self.cli_armar = self.create_client(Armar, 'dron/armar_srv',
                                            callback_group=grupo_datos)

        self.create_subscription(EstadoAltura, 'dron/estado_altura', self.cb_altura,
                                 qos_profile_sensor_data, callback_group=grupo_datos)
        self.create_subscription(EstadoDron, 'dron/estado_dron', self.cb_estado, 10,
                                 callback_group=grupo_datos)

        self.alt = None
        self.estado = None

        self.servidor = ActionServer(
            self, IrAAltura, 'ir_a_altura',
            execute_callback=self.ejecutar_meta,
            goal_callback=self.aceptar_meta,
            cancel_callback=self.aceptar_cancelacion,
            callback_group=grupo_accion)

        if self.publicar_latido:
            self.create_timer(1.0 / FREQ_LATIDO, self.tick_latido,
                              callback_group=grupo_datos)

        if perfil:
            self.perfil = yaml.safe_load(open(ruta_perfil(perfil)))
            self.create_timer(1.0, self.arrancar_perfil, callback_group=grupo_perfil)
            self._perfil_lanzado = False
        else:
            self.perfil = None

        self.get_logger().info('mision_altura lista%s'
                               % (' (perfil %s)' % perfil if perfil else ''))

    # ---------- datos ----------
    def cb_altura(self, msg):
        self.alt = msg

    def cb_estado(self, msg):
        self.estado = msg

    def tick_latido(self):
        self.pub_actitud.publish(Vector3())

    def ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- acción ----------
    def aceptar_meta(self, peticion):
        if peticion.altura <= 0.0:
            self.get_logger().warn('meta rechazada: altura %.2f' % peticion.altura)
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def aceptar_cancelacion(self, _):
        return CancelResponse.ACCEPT

    def ejecutar_meta(self, meta):
        peticion = meta.request
        altura = float(peticion.altura)
        tolerancia = float(peticion.tolerancia) or TOLERANCIA_DEF
        limite = float(peticion.tiempo_limite) or LIMITE_DEF

        resultado = IrAAltura.Result()
        self.pub_altura.publish(Float32(data=altura))
        self.get_logger().info('-> altura %.2f m (tol %.2f, limite %.0f s)'
                               % (altura, tolerancia, limite))

        t0 = self.ahora()
        estables = 0
        realimentacion = IrAAltura.Feedback()

        # Si el dron esta desarmado no va a subir: mejor decirlo ya que esperar a
        # que expire el tiempo limite.
        time.sleep(0.3)
        if self.estado is not None and self.estado.modo == DESARMADO:
            meta.abort()
            resultado.alcanzada = False
            resultado.error_final = self.error(altura)
            resultado.resultado = ('el dron esta desarmado (la altura objetivo queda '
                                   'fijada: arma y repite)')
            self.get_logger().warn('   ' + resultado.resultado)
            return resultado

        while rclpy.ok():
            if meta.is_cancel_requested:
                meta.canceled()
                resultado.alcanzada = False
                resultado.resultado = 'cancelada'
                resultado.error_final = self.error(altura)
                return resultado

            transcurrido = self.ahora() - t0
            if self.alt is not None:
                error = self.error(altura)
                realimentacion.altura_actual = float(self.alt.fusionada)
                realimentacion.error = float(error)
                realimentacion.transcurrido = float(transcurrido)
                meta.publish_feedback(realimentacion)

                estables = estables + 1 if abs(error) <= tolerancia else 0
                if estables >= MUESTRAS_ESTABLE:
                    meta.succeed()
                    resultado.alcanzada = True
                    resultado.error_final = float(error)
                    resultado.resultado = ('alcanzada en %.1f s' % transcurrido)
                    self.get_logger().info('   ' + resultado.resultado)
                    return resultado

            # Si el dron deja de estar armado (failsafe, emergencia), no hay nada
            # que esperar: la meta ha fracasado.
            if self.estado is not None and self.estado.modo in (ATERRIZAJE, DETENIDO):
                meta.abort()
                resultado.alcanzada = False
                resultado.error_final = self.error(altura)
                resultado.resultado = 'el dron paso a %s' % self.estado.descripcion
                self.get_logger().warn('   ' + resultado.resultado)
                return resultado

            if transcurrido > limite:
                meta.abort()
                resultado.alcanzada = False
                resultado.error_final = self.error(altura)
                resultado.resultado = ('no convergio en %.0f s (error %.2f m)'
                                       % (limite, resultado.error_final))
                self.get_logger().warn('   ' + resultado.resultado)
                return resultado

            time.sleep(0.1)

        resultado.alcanzada = False
        resultado.resultado = 'interrumpida'
        return resultado

    def error(self, altura):
        return 0.0 if self.alt is None else float(altura - self.alt.fusionada)

    # ---------- perfil ----------
    def arrancar_perfil(self):
        """El perfil se ejecuta en un hilo aparte: así los callbacks siguen
        atendiéndose mientras el perfil espera."""
        if self._perfil_lanzado:
            return
        self._perfil_lanzado = True
        threading.Thread(target=self.ejecutar_perfil, daemon=True).start()

    def esperar_telemetria(self):
        limite = time.monotonic() + self.get_parameter('espera_telemetria').value
        while time.monotonic() < limite:
            if self.alt is not None and self.estado is not None:
                return True
            time.sleep(0.1)
        return False

    def armar(self):
        if not self.cli_armar.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('no hay servicio dron/armar_srv')
            return False
        futuro = self.cli_armar.call_async(Armar.Request(armar=True))
        limite = time.monotonic() + 5.0
        while not futuro.done() and time.monotonic() < limite:
            time.sleep(0.05)
        if not futuro.done():
            self.get_logger().error('el servicio de armado no respondio')
            return False
        res = futuro.result()
        if not res.aceptado:
            self.get_logger().error('armado rechazado: %s' % res.motivo)
        return res.aceptado

    def ir_a(self, altura, tolerancia, limite):
        """Misma lógica que la acción, pero llamada desde el perfil."""
        self.pub_altura.publish(Float32(data=float(altura)))
        self.get_logger().info('-> altura %.2f m' % altura)
        t0 = time.monotonic()
        estables = 0
        while time.monotonic() - t0 < limite:
            if self.alt is not None:
                if abs(altura - self.alt.fusionada) <= tolerancia:
                    estables += 1
                    if estables >= MUESTRAS_ESTABLE:
                        self.get_logger().info('   alcanzada en %.1f s'
                                               % (time.monotonic() - t0))
                        return True
                else:
                    estables = 0
            if self.estado is not None and self.estado.modo in (ATERRIZAJE, DETENIDO):
                self.get_logger().warn('   el dron paso a %s' % self.estado.descripcion)
                return False
            time.sleep(0.1)
        self.get_logger().warn('   no convergio en %.0f s' % limite)
        return False

    def ejecutar_perfil(self):
        nombre = self.perfil.get('nombre', 'perfil')
        self.get_logger().info('=== perfil: %s ===' % nombre)
        if not self.esperar_telemetria():
            self.get_logger().error('sin telemetria: no arranco el perfil')
            return

        tolerancia = float(self.perfil.get('tolerancia', TOLERANCIA_DEF))
        limite = float(self.perfil.get('limite_por_paso', LIMITE_DEF))

        for paso in self.perfil.get('pasos', []):
            accion = paso.get('accion')
            if accion == 'fijar':
                # Hay que fijar la altura ANTES de armar: el firmware rechaza el
                # armado si no tiene setpoint (y el simulador hace lo mismo).
                self.pub_altura.publish(Float32(data=float(paso['altura'])))
                self.get_logger().info('-> fijar altura objetivo %.2f m' % paso['altura'])
                time.sleep(0.5)
            elif accion == 'armar':
                if not self.armar():
                    return
            elif accion == 'ir':
                if not self.ir_a(paso['altura'], tolerancia, limite):
                    return
            elif accion == 'mantener':
                self.get_logger().info('-> mantener %.1f s' % paso['segundos'])
                time.sleep(float(paso['segundos']))
            elif accion == 'aterrizar':
                self.get_logger().info('-> aterrizaje por emergencia controlada')
                self.pub_emergencia.publish(Bool(data=True))
                # El firmware baja a VELOCIDAD_DESCENSO y corta al detectar suelo
                limite_aterrizaje = time.monotonic() + 30.0
                while time.monotonic() < limite_aterrizaje:
                    if self.estado is not None and self.estado.modo == DETENIDO:
                        self.get_logger().info('   en el suelo, motores cortados')
                        break
                    time.sleep(0.2)
            else:
                self.get_logger().warn('accion desconocida en el perfil: %r' % accion)

        self.get_logger().info('=== perfil %s terminado ===' % nombre)


def main():
    rclpy.init()
    nodo = MisionAltura()
    ejecutor = MultiThreadedExecutor(num_threads=4)
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
