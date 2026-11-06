#!/usr/bin/env python3
"""Leyes de control del dron: PID, cascada de actitud, cascada de altura y mezcla.

Puerto en Python de `src/pid.h`, `src/control_actitud.cpp`,
`src/control_vuelo.cpp` y `src/motores.cpp` del firmware. Es deliberadamente un
reflejo: si se cambia una ley en el firmware hay que cambiarla aquí, porque la
simulación solo vale como banco de pruebas mientras las dos coincidan.

El control de altura sigue el esquema del Lab_rav_03 (sjtu_drone): cascada
posición -> velocidad con feedforward de gravedad.

La estimación de estado vive en robot_estimation.estimacion.
"""

G = 9.80665


def acotar(x, minimo, maximo):
    return minimo if x < minimo else (maximo if x > maximo else x)


# ===================== PARÁMETROS (espejo de src/config.h) =====================


class ParametrosControl:

    # --- Actitud: lazo externo (ángulo -> deg/s) e interno (deg/s -> throttle) ---
    KP_ANG = 4.5
    RATE_SP_MAX = 150.0
    KP_RATE = 0.0020
    KI_RATE = 0.0040
    KD_RATE = 0.00006
    KP_YAW = 0.0040
    KI_YAW = 0.0060
    KD_YAW = 0.0
    CORR_MAX = 0.35
    I_MAX_ACTITUD = 0.10
    ANG_SP_MAX = 15.0
    YAW_RATE_SP_MAX = 90.0

    # --- Altura: cascada PID ---
    # Ganancias de partida del Lab_rav_03: positionZ kp=1.0 kd=0.2,
    # velocityZ kp=5.0 kd=1.0.
    ALTURA_SP_MAX = 3.0

    KP_POS_Z = 1.0
    KI_POS_Z = 0.0
    KD_POS_Z = 0.2
    TAU_POS_Z = 0.20
    VEL_Z_MAX = 0.50

    KP_VEL_Z = 5.0
    KI_VEL_Z = 1.0          # el laboratorio usa 0; aquí absorbe el error de hover
    KD_VEL_Z = 1.0
    TAU_VEL_Z = 0.05
    I_MAX_VEL_Z = 3.0
    ACC_Z_MAX = 4.0

    THROTTLE_HOVER = 0.36

    # --- Motores ---
    MOTOR_IDLE = 0.08
    RAMP_TIME_S = 0.8
    PWM_MIN = 1638          # cuentas a 400 Hz y 12 bits (1000 us)
    PWM_MAX = 3277          # cuentas a 400 Hz y 12 bits (2000 us)

    # --- Failsafe ---
    TIMEOUT_CMD_S = 0.5
    TIMEOUT_IMU_S = 0.2
    VELOCIDAD_DESCENSO = 0.4
    ALTURA_CORTE_MOTORES = 0.10


# ===================== PID =====================


class PID:
    """Misma semántica que el PIDController del Lab_rav_03 y que src/pid.h:

        consigna_filtrada = (dt*consigna + tau*consigna_filtrada) / (dt + tau)
        p = consigna_filtrada - x
        d = dconsigna - dx          <- derivada MEDIDA, no la del error
        i = i + dt*p
        salida = kp*p + kd*d + ki*i

    Con dos añadidos: integrador acotado (el original no tiene anti-windup) y
    salida acotada con integración condicional.
    """

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, tau=0.0,
                 limite=-1.0, i_max=-1.0, salida_max=-1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.tau = tau
        self.limite = limite
        self.i_max = i_max
        self.salida_max = salida_max
        self.reset()

    def reset(self):
        self.consigna = 0.0
        self.dconsigna = 0.0
        self.p = self.i = self.d = 0.0
        self.salida = 0.0

    def inicializar_consigna(self, valor):
        """Arranca el filtro en un valor concreto (p. ej. la altura actual al
        armar), para que la consigna no entre como un escalón desde cero."""
        self.consigna = valor
        self.dconsigna = 0.0

    def actualizar(self, consigna_nueva, x, dx, dt):
        if dt <= 0.0:
            return self.salida

        if self.limite > 0.0:
            consigna_nueva = acotar(consigna_nueva, -self.limite, self.limite)

        if dt + self.tau > 0.0:
            previa = self.consigna
            self.consigna = (dt * consigna_nueva + self.tau * self.consigna) / (dt + self.tau)
            self.dconsigna = (self.consigna - previa) / dt

        self.p = self.consigna - x
        self.d = self.dconsigna - dx

        candidato = self.i + dt * self.p
        if self.i_max > 0.0:
            candidato = acotar(candidato, -self.i_max, self.i_max)
        tentativa = self.kp * self.p + self.kd * self.d + self.ki * candidato
        saturado = self.salida_max > 0.0 and (
            (tentativa > self.salida_max and self.p > 0.0)
            or (tentativa < -self.salida_max and self.p < 0.0))
        if not saturado:
            self.i = candidato

        self.salida = self.kp * self.p + self.kd * self.d + self.ki * self.i
        if self.salida_max > 0.0:
            self.salida = acotar(self.salida, -self.salida_max, self.salida_max)
        return self.salida


# ===================== CONTROL DE ACTITUD (CASCADA) =====================


class ControlActitud:
    """Lazo externo de ángulo + lazo interno de velocidad angular.

    El lazo interno usa la medida directa del giróscopo y pone el término D sobre
    la medida, no sobre el error: así un escalón en la consigna no produce un
    pico de derivada. La guiñada no tiene lazo de ángulo (sin magnetómetro el yaw
    deriva sin cota): se controla velocidad de guiñada.
    """

    def __init__(self, par=None):
        self.par = par or ParametrosControl()
        self.kp_ang = self.par.KP_ANG
        self.kp_rate = self.par.KP_RATE
        self.ki_rate = self.par.KI_RATE
        self.kd_rate = self.par.KD_RATE
        self.kp_yaw = self.par.KP_YAW
        self.ki_yaw = self.par.KI_YAW

        self.sp_roll = 0.0
        self.sp_pitch = 0.0
        self.sp_yaw_rate = 0.0

        self.corr_roll = 0.0
        self.corr_pitch = 0.0
        self.corr_yaw = 0.0
        self.rate_sp_roll = 0.0
        self.rate_sp_pitch = 0.0

        self._i = [0.0, 0.0, 0.0]
        self._vel_prev = [0.0, 0.0]

    def reset(self, vel_roll=0.0, vel_pitch=0.0):
        self._i = [0.0, 0.0, 0.0]
        self._vel_prev = [vel_roll, vel_pitch]
        self.corr_roll = self.corr_pitch = self.corr_yaw = 0.0

    def ganancias(self):
        return [self.kp_ang, self.kp_rate, self.ki_rate,
                self.kd_rate, self.kp_yaw, self.ki_yaw]

    def set_ganancias(self, valores):
        (self.kp_ang, self.kp_rate, self.ki_rate,
         self.kd_rate, self.kp_yaw, self.ki_yaw) = valores[:6]
        self.reset()

    def _pid_velocidad(self, rate_sp, rate, rate_prev, kp, ki, kd, idx, dt):
        error = rate_sp - rate
        p = kp * error
        d = kd * (rate_prev - rate) / dt
        sin_i = p + d

        candidato = acotar(self._i[idx] + ki * error * dt,
                           -self.par.I_MAX_ACTITUD, self.par.I_MAX_ACTITUD)
        salida = sin_i + candidato
        saturado_arriba = salida > self.par.CORR_MAX and error > 0
        saturado_abajo = salida < -self.par.CORR_MAX and error < 0
        if not (saturado_arriba or saturado_abajo):
            self._i[idx] = candidato

        return acotar(sin_i + self._i[idx], -self.par.CORR_MAX, self.par.CORR_MAX)

    def actualizar(self, est, dt):
        """`est` es un EstimadorActitud de robot_estimation."""
        if dt <= 0.0:
            return

        sp_roll = acotar(self.sp_roll, -self.par.ANG_SP_MAX, self.par.ANG_SP_MAX)
        sp_pitch = acotar(self.sp_pitch, -self.par.ANG_SP_MAX, self.par.ANG_SP_MAX)

        self.rate_sp_roll = acotar(self.kp_ang * (sp_roll - est.roll),
                                   -self.par.RATE_SP_MAX, self.par.RATE_SP_MAX)
        self.rate_sp_pitch = acotar(self.kp_ang * (sp_pitch - est.pitch),
                                    -self.par.RATE_SP_MAX, self.par.RATE_SP_MAX)

        self.corr_roll = self._pid_velocidad(
            self.rate_sp_roll, est.vel_roll, self._vel_prev[0],
            self.kp_rate, self.ki_rate, self.kd_rate, 0, dt)
        self.corr_pitch = self._pid_velocidad(
            self.rate_sp_pitch, est.vel_pitch, self._vel_prev[1],
            self.kp_rate, self.ki_rate, self.kd_rate, 1, dt)

        sp_yaw = acotar(self.sp_yaw_rate,
                        -self.par.YAW_RATE_SP_MAX, self.par.YAW_RATE_SP_MAX)
        self.corr_yaw = self._pid_velocidad(
            sp_yaw, est.vel_yaw, est.vel_yaw,
            self.kp_yaw, self.ki_yaw, self.par.KD_YAW, 2, dt)

        self._vel_prev = [est.vel_roll, est.vel_pitch]


# ===================== CONTROL DE ALTURA (CASCADA PID) =====================


class ControlAltura:
    """Cascada posición -> velocidad con feedforward de gravedad.

        vz_deseada  = pos_z(objetivo, altura, vz)
        acc_deseada = vel_z(vz_deseada, vz, acc_vertical)
        throttle    = THROTTLE_HOVER * (1 + acc_deseada / g)

    La última línea es el equivalente normalizado del `force = masa * (pid + g)`
    del plugin del Lab_rav_03: con aceleración deseada nula sale exactamente el
    throttle de hover, y el integral del PID de velocidad absorbe el error de
    estimación de ese hover (peso real, batería, hélices).
    """

    def __init__(self, par=None):
        self.par = par or ParametrosControl()
        self.objetivo = 1.0
        self.pos_z = PID(self.par.KP_POS_Z, self.par.KI_POS_Z, self.par.KD_POS_Z,
                         self.par.TAU_POS_Z, limite=self.par.ALTURA_SP_MAX,
                         salida_max=self.par.VEL_Z_MAX)
        self.vel_z = PID(self.par.KP_VEL_Z, self.par.KI_VEL_Z, self.par.KD_VEL_Z,
                         self.par.TAU_VEL_Z, limite=self.par.VEL_Z_MAX,
                         i_max=self.par.I_MAX_VEL_Z, salida_max=self.par.ACC_Z_MAX)
        self.throttle_base = self.par.THROTTLE_HOVER

    @property
    def consigna(self):
        return self.pos_z.consigna

    @property
    def vz_deseada(self):
        return self.pos_z.salida

    @property
    def acc_deseada(self):
        return self.vel_z.salida

    def reset(self, altura_actual):
        self.pos_z.reset()
        self.vel_z.reset()
        self.pos_z.inicializar_consigna(altura_actual)
        self.throttle_base = self.par.THROTTLE_HOVER

    def set_ganancias_altura(self, kp_pos, kd_pos, kp_vel, ki_vel, kd_vel):
        self.pos_z.kp, self.pos_z.kd = kp_pos, kd_pos
        self.vel_z.kp, self.vel_z.ki, self.vel_z.kd = kp_vel, ki_vel, kd_vel

    def actualizar(self, altura, vz, acc_vertical, dt):
        vz_deseada = self.pos_z.actualizar(self.objetivo, altura, vz, dt)
        acc_deseada = self.vel_z.actualizar(vz_deseada, vz, acc_vertical, dt)
        self.throttle_base = acotar(
            self.par.THROTTLE_HOVER * (1.0 + acc_deseada / G), 0.0, 1.0)
        return self.throttle_base


# ===================== MEZCLA =====================


def mezclar(throttle, corr_roll, corr_pitch, corr_yaw, par=None):
    """Mezcla en X con desaturación por desplazamiento común.

    Si algún motor se sale del rango se desplaza el throttle común de los cuatro
    en vez de recortar uno solo: recortar destruye el par diferencial justo
    cuando el control más lo necesita.

    Devuelve los cuatro mandos normalizados [MOTOR_IDLE, 1] en el orden
    M1 frente-izq, M2 frente-der, M3 tras-der, M4 tras-izq.
    """
    par = par or ParametrosControl()

    m = [
        throttle + corr_roll - corr_pitch - corr_yaw,
        throttle - corr_roll - corr_pitch + corr_yaw,
        throttle - corr_roll + corr_pitch - corr_yaw,
        throttle + corr_roll + corr_pitch + corr_yaw,
    ]

    mx, mn = max(m), min(m)
    if mx > 1.0:
        desplazamiento = 1.0 - mx
    elif mn < par.MOTOR_IDLE:
        desplazamiento = par.MOTOR_IDLE - mn
    else:
        desplazamiento = 0.0

    return [acotar(v + desplazamiento, par.MOTOR_IDLE, 1.0) for v in m]
