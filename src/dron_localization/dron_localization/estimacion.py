#!/usr/bin/env python3
"""Estimación de estado: actitud y altura.

Puerto en Python de lo que hace el firmware en `src/sensores.cpp`. Es
deliberadamente un reflejo: mismo filtro complementario, misma fusión
LiDAR/barómetro con anclaje, misma estimación de velocidad y aceleración
vertical. Si cambia el firmware, hay que cambiar esto para que la simulación
siga sirviendo de banco de pruebas.

Está en dron_localization y no en dron_control porque es estimación de estado,
no ley de control: dron_control importa de aquí, nunca al revés.

Convenciones de signos (REP-103: X adelante, Y a la izquierda, Z arriba), iguales
a las del firmware:
  - roll positivo  = alabeo a la derecha
  - pitch positivo = morro abajo
  - yaw positivo   = giro antihorario visto desde arriba
"""

import math

G = 9.80665


def acotar(x, minimo, maximo):
    return minimo if x < minimo else (maximo if x > maximo else x)


class ParametrosEstimacion:
    """Espejo de la sección de estimación de src/config.h."""

    FILTRO_COMP_TAU = 0.50      # s
    GYRO_LPF_HZ = 40.0
    VZ_LPF_HZ = 2.0
    ACC_Z_LPF_HZ = 5.0

    ALTURA_LIDAR_MAX = 1.5      # m, hasta aquí manda el LiDAR
    ALTURA_MEZCLA_MIN = 1.0     # m, desde aquí se mezcla con el barómetro
    VZ_MAX_CRUDA = 3.0          # m/s, acota la derivada antes de filtrarla

    # Ganancia del filtro complementario de velocidad vertical [1/s]. La
    # predicción la da la aceleración de la IMU y la corrección la derivada de la
    # altura. Derivar solo el barómetro (sigma ~8 cm a 10 Hz) daba ~0.4 m/s de
    # ruido en vz, que con kp_vel_z=5 son 2 m/s^2 de basura en el mando: medido
    # en simulación, el dron oscilaba +-0.2 m por encima del alcance del LiDAR.
    K_VZ = 1.5


class EstimadorActitud:
    """Filtro complementario alimentado por la IMU."""

    def __init__(self, par=None):
        self.par = par or ParametrosEstimacion()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.vel_roll = 0.0        # deg/s filtrados
        self.vel_pitch = 0.0
        self.vel_yaw = 0.0
        self.acc_vertical = 0.0    # m/s^2, sin gravedad

    def actualizar(self, acc, gyr, dt):
        """acc en g, gyr en deg/s, ambos en ejes del cuerpo."""
        if dt <= 0.0:
            return

        alpha_g = dt / (dt + 1.0 / (2.0 * math.pi * self.par.GYRO_LPF_HZ))
        self.vel_roll += alpha_g * (gyr[0] - self.vel_roll)
        self.vel_pitch += alpha_g * (gyr[1] - self.vel_pitch)
        self.vel_yaw += alpha_g * (gyr[2] - self.vel_yaw)

        acc_roll = math.degrees(math.atan2(acc[1], acc[2]))
        acc_pitch = math.degrees(
            math.atan2(-acc[0], math.sqrt(acc[1] * acc[1] + acc[2] * acc[2])))

        alpha_c = self.par.FILTRO_COMP_TAU / (self.par.FILTRO_COMP_TAU + dt)
        self.roll = alpha_c * (self.roll + gyr[0] * dt) + (1.0 - alpha_c) * acc_roll
        self.pitch = alpha_c * (self.pitch + gyr[1] * dt) + (1.0 - alpha_c) * acc_pitch

        # Aceleración vertical: se proyecta la medida y se le quita la gravedad.
        # Alimenta el término D del PID de velocidad vertical.
        az_mundo = (acc[2] * math.cos(math.radians(self.roll))
                    * math.cos(math.radians(self.pitch)) - 1.0) * G
        alpha_a = dt / (dt + 1.0 / (2.0 * math.pi * self.par.ACC_Z_LPF_HZ))
        self.acc_vertical += alpha_a * (az_mundo - self.acc_vertical)

        # Sin magnetómetro el yaw deriva sin cota: se publica como referencia,
        # pero el control de guiñada trabaja sobre velocidad, no sobre ángulo.
        self.yaw += gyr[2] * dt
        if self.yaw > 180.0:
            self.yaw -= 360.0
        elif self.yaw < -180.0:
            self.yaw += 360.0

    def cuaternion(self):
        """Cuaternión (x, y, z, w) de la actitud estimada."""
        cr = math.cos(math.radians(self.roll) * 0.5)
        sr = math.sin(math.radians(self.roll) * 0.5)
        cp = math.cos(math.radians(self.pitch) * 0.5)
        sp = math.sin(math.radians(self.pitch) * 0.5)
        cy = math.cos(math.radians(self.yaw) * 0.5)
        sy = math.sin(math.radians(self.yaw) * 0.5)
        return (sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy)


class EstimadorAltura:
    """Fusión LiDAR + barómetro con anclaje del barómetro al LiDAR."""

    def __init__(self, par=None):
        self.par = par or ParametrosEstimacion()
        self.lidar = 0.0
        self.lidar_valido = False
        self.baro = 0.0
        self.baro_inicializado = False
        self.correccion_baro = 0.0
        self.altura = 0.0
        self.vz = 0.0
        self._altura_prev = None

    def set_lidar(self, metros, valido):
        self.lidar_valido = valido
        if valido:
            self.lidar = metros

    def set_baro(self, metros):
        self.baro = metros
        self.baro_inicializado = True

    def actualizar(self, dt, acc_vertical=0.0):
        par = self.par

        # Mientras el LiDAR es la referencia buena se estima la deriva del
        # barómetro. Sin este anclaje, salir del alcance del LiDAR mete un
        # escalón de decenas de centímetros en la altura estimada.
        if (self.lidar_valido and self.lidar < par.ALTURA_MEZCLA_MIN
                and self.baro_inicializado):
            self.correccion_baro += 0.05 * ((self.lidar - self.baro) - self.correccion_baro)
        baro_corregido = self.baro + self.correccion_baro

        if self.lidar_valido and self.lidar < par.ALTURA_LIDAR_MAX:
            if self.lidar > par.ALTURA_MEZCLA_MIN:
                k = ((self.lidar - par.ALTURA_MEZCLA_MIN)
                     / (par.ALTURA_LIDAR_MAX - par.ALTURA_MEZCLA_MIN))
                altura = (1.0 - k) * self.lidar + k * baro_corregido
            else:
                altura = self.lidar
        else:
            altura = baro_corregido

        if self._altura_prev is None:
            self._altura_prev = altura
        if dt > 0.0:
            # Filtro complementario de velocidad vertical: la IMU predice y la
            # altura corrige. Mismo principio que el filtro de actitud, y por el
            # mismo motivo: la derivada de la altura es ruidosa a corto plazo y
            # la integral de la aceleración deriva a largo.
            vz_medida = acotar((altura - self._altura_prev) / dt,
                               -par.VZ_MAX_CRUDA, par.VZ_MAX_CRUDA)
            self.vz += acc_vertical * dt
            self.vz += par.K_VZ * (vz_medida - self.vz) * dt
            self.vz = acotar(self.vz, -par.VZ_MAX_CRUDA, par.VZ_MAX_CRUDA)
        self._altura_prev = altura

        self.altura = altura
        return altura
