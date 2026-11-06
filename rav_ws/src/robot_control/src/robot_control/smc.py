#!/usr/bin/env python3
"""Control robusto por modo deslizante (SMC) de altura y actitud.

Puerto en Python de `src/smc.h` del firmware, igual que control.py lo es del PID:
si se cambia una ley en el firmware hay que cambiarla aquí.

Es el SMC con modelo aproximado de las notas del curso (sección 4.2, ec. 16), con
la saturación en lugar del signo para reducir el chattering:

    e  = q - qd
    s  = de + lambda * e
    qr_punto  = qd_punto  - lambda * e
    qr_2punto = qd_2punto - lambda * de
    u  = tau_eq_estimado - kd * s - rho * sat(s / phi)

El modelo estimado es el del dron (Robot3D.py), un grado de libertad por eje:

    altura:  m * z_2punto + m * g = T * cos(roll) * cos(pitch)
    roll:    Ix * p_punto + q * r * (Iz - Iy) = tau_x
    pitch:   Iy * q_punto + p * r * (Ix - Iz) = tau_y
    guiñada: Iz * r_punto + p * q * (Iy - Ix) = tau_z

La guiñada es de primer orden porque, sin magnetómetro, solo se controla su
velocidad: s = r - r_deseada.

rho tiene que superar la cota de la perturbación más el error de modelo (ec. de
las notas: rho_i > D_i + delta_tau_i). La pendiente de la ley dentro de la capa
límite, kd + rho / phi, no es libre: junto con el retardo del motor decide si el
lazo oscila. Por eso phi es mucho mayor que el 0.05 de ejemplo de las notas.

El precio de la capa límite es que una perturbación constante D deja
s = D / (kd + rho / phi), no cero. En altura eso es solo un error de régimen, pero
en actitud es grave: en vuelo el acelerómetro no ve la inclinación, el filtro
complementario lleva el ángulo estimado a cero y ese s se vuelve una velocidad de
giro constante, que tumba al dron (medido en Gazebo con 0.10 N*m en roll). Un centro
de masa desplazado o dos motores desiguales son justo eso.

Por eso cada eje estima además ese sesgo constante con la ley adaptativa del curso
(notebook 02-1) con regresor Y = 1:

    u = tau_eq_estimado + sesgo - kd * s - rho * sat(s / phi)
    sesgo_punto = -p * s

La adaptación solo corre dentro de la capa límite: fuera, rho ya empuja hacia la
superficie, e integrar ahí acumularía en tierra y en los transitorios grandes.
"""

import math

from robot_control.control import G, ParametrosControl, acotar


class ParametrosSMC:
    """Espejo de la sección SMC de src/config.h."""

    # --- Modelo estimado (robot_description/config/f450.yaml) ---
    MASA = 1.15
    IXX = 0.0115
    IYY = 0.0115
    IZZ = 0.0218
    BRAZO = 0.1589          # m, brazo de palanca de cada motor en X y en Y
    EMPUJE_MAX = 8.3        # N por motor
    K_YAW = 0.016           # m, par de reacción / empuje

    # --- Roll y pitch ---
    LAMBDA_ANG = 4.5        # 1/s
    KD_ANG = 0.25           # N*m por rad/s
    RHO_ANG = 0.15          # N*m
    PHI_ANG = 0.50          # rad/s
    P_ANG = 1.0             # N*m por rad, ganancia de adaptación
    SESGO_MAX_ANG = 0.30    # N*m

    # --- Guiñada ---
    KD_YAW = 0.10           # N*m por rad/s
    RHO_YAW = 0.03          # N*m
    PHI_YAW = 0.50          # rad/s
    P_YAW = 0.2             # N*m por rad
    SESGO_MAX_YAW = 0.10    # N*m

    # --- Altura ---
    LAMBDA_Z = 1.2          # 1/s
    KD_Z = 3.0              # N por m/s
    RHO_Z = 2.5             # N
    PHI_Z = 0.50            # m/s
    P_Z = 3.0               # N por m
    SESGO_MAX_Z = 3.0       # N

    # --- Generador de referencia de altura ---
    OMEGA_REF_Z = 2.0       # rad/s, segundo orden críticamente amortiguado
    ACC_REF_MAX = 2.0       # m/s^2


def sat(x, phi):
    """sat(x / phi) de las notas: lineal en la capa límite, +-1 fuera."""
    return acotar(x / phi, -1.0, 1.0)


def ley_smc(tau_eq, s, kd, rho, phi):
    """Ec. (16) de las notas."""
    return tau_eq - kd * s - rho * sat(s, phi)


def adaptar(sesgo, s, phi, p, maximo, dt):
    """Ley adaptativa con Y = 1, solo dentro de la capa límite."""
    if abs(s) >= phi:
        return sesgo
    return acotar(sesgo - p * s * dt, -maximo, maximo)


# ===================== ACTITUD =====================


class ControlActitudSMC:
    """Misma interfaz que control.ControlActitud: consignas y correcciones de
    throttle en las mismas unidades, para que la mezcla no cambie."""

    def __init__(self, par=None, smc=None):
        self.par = par or ParametrosControl()
        self.smc = smc or ParametrosSMC()

        self.sp_roll = 0.0
        self.sp_pitch = 0.0
        self.sp_yaw_rate = 0.0

        self.corr_roll = 0.0
        self.corr_pitch = 0.0
        self.corr_yaw = 0.0
        self.rate_sp_roll = 0.0
        self.rate_sp_pitch = 0.0

        self.s_roll = 0.0
        self.s_pitch = 0.0
        self.s_yaw = 0.0
        self.sesgo = [0.0, 0.0, 0.0]

    def reset(self, vel_roll=0.0, vel_pitch=0.0):
        self.corr_roll = self.corr_pitch = self.corr_yaw = 0.0
        self.s_roll = self.s_pitch = self.s_yaw = 0.0
        self.sesgo = [0.0, 0.0, 0.0]

    def _eje(self, angulo, sp, rate, inercia, giroscopico, indice, dt):
        """Devuelve (tau, s, velocidad de referencia), todo en rad y N*m."""
        p = self.smc
        e = angulo - sp
        # La consigna de ángulo llega como escalón: qd_punto = qd_2punto = 0
        vel_ref = -p.LAMBDA_ANG * e
        acc_ref = -p.LAMBDA_ANG * rate
        s = rate - vel_ref
        self.sesgo[indice] = adaptar(self.sesgo[indice], s, p.PHI_ANG, p.P_ANG,
                                     p.SESGO_MAX_ANG, dt)
        tau_eq = inercia * acc_ref + giroscopico + self.sesgo[indice]
        return ley_smc(tau_eq, s, p.KD_ANG, p.RHO_ANG, p.PHI_ANG), s, vel_ref

    def actualizar(self, est, dt):
        """`est` es un EstimadorActitud de robot_estimation."""
        if dt <= 0.0:
            return

        p = self.smc
        roll = math.radians(est.roll)
        pitch = math.radians(est.pitch)
        vel_p = math.radians(est.vel_roll)
        vel_q = math.radians(est.vel_pitch)
        vel_r = math.radians(est.vel_yaw)
        sp_roll = math.radians(acotar(self.sp_roll, -self.par.ANG_SP_MAX,
                                      self.par.ANG_SP_MAX))
        sp_pitch = math.radians(acotar(self.sp_pitch, -self.par.ANG_SP_MAX,
                                       self.par.ANG_SP_MAX))
        sp_yaw = math.radians(acotar(self.sp_yaw_rate, -self.par.YAW_RATE_SP_MAX,
                                     self.par.YAW_RATE_SP_MAX))

        tau_x, self.s_roll, ref_roll = self._eje(
            roll, sp_roll, vel_p, p.IXX, vel_q * vel_r * (p.IZZ - p.IYY), 0, dt)
        tau_y, self.s_pitch, ref_pitch = self._eje(
            pitch, sp_pitch, vel_q, p.IYY, vel_p * vel_r * (p.IXX - p.IZZ), 1, dt)

        self.s_yaw = vel_r - sp_yaw
        self.sesgo[2] = adaptar(self.sesgo[2], self.s_yaw, p.PHI_YAW, p.P_YAW,
                                p.SESGO_MAX_YAW, dt)
        tau_z = ley_smc(vel_p * vel_q * (p.IYY - p.IXX) + self.sesgo[2], self.s_yaw,
                        p.KD_YAW, p.RHO_YAW, p.PHI_YAW)

        # La mezcla reparte la corrección en los cuatro motores: una corrección c
        # produce 4 * c * EMPUJE_MAX de empuje diferencial.
        por_corr = 4.0 * p.EMPUJE_MAX
        self.corr_roll = acotar(tau_x / (por_corr * p.BRAZO),
                                -self.par.CORR_MAX, self.par.CORR_MAX)
        self.corr_pitch = acotar(tau_y / (por_corr * p.BRAZO),
                                 -self.par.CORR_MAX, self.par.CORR_MAX)
        self.corr_yaw = acotar(tau_z / (por_corr * p.K_YAW),
                               -self.par.CORR_MAX, self.par.CORR_MAX)

        self.rate_sp_roll = math.degrees(ref_roll)
        self.rate_sp_pitch = math.degrees(ref_pitch)


# ===================== ALTURA =====================


class ControlAlturaSMC:
    """Misma interfaz que control.ControlAltura.

    El SMC necesita la trayectoria completa (zd, zd_punto, zd_2punto) y la consigna
    llega como escalón, así que se genera con un filtro de segundo orden acotado en
    velocidad y aceleración.

    El empuje se normaliza con THROTTLE_HOVER, no con EMPUJE_MAX: es el número que
    se calibra en banco, y su error lo absorbe el sesgo estimado.
    """

    def __init__(self, par=None, smc=None):
        self.par = par or ParametrosControl()
        self.smc = smc or ParametrosSMC()
        self.objetivo = 1.0
        self.throttle_base = self.par.THROTTLE_HOVER
        self.consigna = 0.0
        self.consigna_vel = 0.0
        self.consigna_acc = 0.0
        self.vz_deseada = 0.0
        self.acc_deseada = 0.0
        self.s = 0.0
        self.sesgo = 0.0

    def reset(self, altura_actual):
        self.consigna = altura_actual
        self.consigna_vel = 0.0
        self.consigna_acc = 0.0
        self.vz_deseada = 0.0
        self.acc_deseada = 0.0
        self.s = 0.0
        self.sesgo = 0.0
        self.throttle_base = self.par.THROTTLE_HOVER

    def _generar_referencia(self, dt):
        p = self.smc
        objetivo = acotar(self.objetivo, -self.par.ALTURA_SP_MAX, self.par.ALTURA_SP_MAX)
        acc = (p.OMEGA_REF_Z ** 2 * (objetivo - self.consigna)
               - 2.0 * p.OMEGA_REF_Z * self.consigna_vel)
        self.consigna_acc = acotar(acc, -p.ACC_REF_MAX, p.ACC_REF_MAX)
        vel = self.consigna_vel + self.consigna_acc * dt
        self.consigna_vel = acotar(vel, -self.par.VEL_Z_MAX, self.par.VEL_Z_MAX)
        self.consigna += self.consigna_vel * dt

    def actualizar(self, altura, vz, dt, cos_inclinacion=1.0):
        if dt <= 0.0:
            return self.throttle_base

        p = self.smc
        self._generar_referencia(dt)

        e = altura - self.consigna
        de = vz - self.consigna_vel
        self.vz_deseada = self.consigna_vel - p.LAMBDA_Z * e
        acc_ref = self.consigna_acc - p.LAMBDA_Z * de
        self.s = vz - self.vz_deseada

        self.sesgo = adaptar(self.sesgo, self.s, p.PHI_Z, p.P_Z, p.SESGO_MAX_Z, dt)
        fuerza = ley_smc(p.MASA * (acc_ref + G) + self.sesgo, self.s,
                         p.KD_Z, p.RHO_Z, p.PHI_Z)
        self.acc_deseada = acotar(fuerza / p.MASA - G,
                                  -self.par.ACC_Z_MAX, self.par.ACC_Z_MAX)

        # Inclinado, solo la componente vertical del empuje sostiene al dron
        c = max(cos_inclinacion, 0.5)
        self.throttle_base = acotar(
            self.par.THROTTLE_HOVER * (1.0 + self.acc_deseada / G) / c, 0.0, 1.0)
        return self.throttle_base
