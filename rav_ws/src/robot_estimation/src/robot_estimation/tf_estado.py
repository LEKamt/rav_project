#!/usr/bin/env python3
"""Publica el estado estimado como TF y odometría, para RViz y para registro.

Toma lo que publica el controlador (sea el firmware o el de simulación) y lo
convierte en:

    TF odom -> base_link
    /dron/odometria (nav_msgs/Odometry)

Importante: **no hay estimación de posición horizontal**. El dron solo conoce su
altura y su actitud, así que x e y se publican a cero y la covarianza de esos dos
ejes se marca como desconocida (-1 en la diagonal). Si en RViz el dron sube y
baja pero nunca se desplaza, no es un fallo: es exactamente lo que el dron sabe
de sí mismo.
"""

import math

import rclpy
from robot_interfaces.msg import EstadoAltura
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

DESCONOCIDA = -1.0


class TfEstado(Node):

    def __init__(self):
        super().__init__('tf_estado')

        self.declare_parameter('marco_fijo', 'odom')
        self.declare_parameter('marco_base', 'base_link')
        self.declare_parameter('frecuencia', 50.0)
        self.marco_fijo = self.get_parameter('marco_fijo').value
        self.marco_base = self.get_parameter('marco_base').value
        frecuencia = self.get_parameter('frecuencia').value

        self.altura = 0.0
        self.vz = 0.0
        self.orientacion = (0.0, 0.0, 0.0, 1.0)     # x, y, z, w
        self.vel_angular = (0.0, 0.0, 0.0)
        self.hay_datos = False

        self.create_subscription(Imu, 'dron/imu', self.cb_imu, qos_profile_sensor_data)
        self.create_subscription(EstadoAltura, 'dron/estado_altura', self.cb_altura,
                                 qos_profile_sensor_data)

        self.pub_odom = self.create_publisher(Odometry, 'dron/odometria', 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(1.0 / frecuencia, self.publicar)

    def cb_imu(self, msg):
        q = msg.orientation
        self.orientacion = (q.x, q.y, q.z, q.w)
        self.vel_angular = (msg.angular_velocity.x, msg.angular_velocity.y,
                            msg.angular_velocity.z)
        self.hay_datos = True

    def cb_altura(self, msg):
        self.altura = msg.fusionada
        self.vz = msg.velocidad_vertical
        self.hay_datos = True

    def publicar(self):
        if not self.hay_datos:
            return
        ahora = self.get_clock().now().to_msg()
        x, y, z, w = self.orientacion

        t = TransformStamped()
        t.header.stamp = ahora
        t.header.frame_id = self.marco_fijo
        t.child_frame_id = self.marco_base
        t.transform.translation.z = float(self.altura)
        t.transform.rotation.x = float(x)
        t.transform.rotation.y = float(y)
        t.transform.rotation.z = float(z)
        t.transform.rotation.w = float(w)
        self.tf.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = ahora
        odom.header.frame_id = self.marco_fijo
        odom.child_frame_id = self.marco_base
        odom.pose.pose.position.z = float(self.altura)
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.z = float(self.vz)
        odom.twist.twist.angular.x = float(self.vel_angular[0])
        odom.twist.twist.angular.y = float(self.vel_angular[1])
        odom.twist.twist.angular.z = float(self.vel_angular[2])

        # x, y y el yaw absoluto no se estiman: sin magnetómetro ni posición
        # horizontal, marcarlos como desconocidos es más honesto que publicar 0
        # con covarianza pequeña.
        odom.pose.covariance[0] = DESCONOCIDA      # x
        odom.pose.covariance[7] = DESCONOCIDA      # y
        odom.pose.covariance[14] = 0.01            # z: el LiDAR da ~1 cm
        odom.pose.covariance[35] = DESCONOCIDA     # yaw
        self.pub_odom.publish(odom)


def main():
    rclpy.init()
    nodo = TfEstado()
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
