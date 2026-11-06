"""Pruebas del SMC: la ley de las notas (ec. 16) y su comportamiento en lazo cerrado.

Los lazos cerrados usan un modelo vertical de un grado de libertad con el mismo
retardo de motor que la simulación: la robustez que promete la teoría solo cuenta
si sobrevive a ese retardo.
"""

import math

import pytest
from robot_control.control import G, ParametrosControl
from robot_control.smc import (ControlActitudSMC, ControlAlturaSMC, ParametrosSMC,
                               adaptar, ley_smc, sat)

PAR = ParametrosControl()
SMC = ParametrosSMC()
TAU_MOTOR = 0.08
EMPUJE_TOTAL = 4 * 8.3


class Est:
    """Lo que ControlActitudSMC lee de un EstimadorActitud."""

    def __init__(self, roll=0.0, pitch=0.0, vel_roll=0.0, vel_pitch=0.0, vel_yaw=0.0):
        self.roll, self.pitch = roll, pitch
        self.vel_roll, self.vel_pitch, self.vel_yaw = vel_roll, vel_pitch, vel_yaw


def sin_adaptacion():
    smc = ParametrosSMC()
    smc.P_ANG = smc.P_YAW = smc.P_Z = 0.0
    return smc


def volar_altura(masa_real=SMC.MASA, fuerza_externa=0.0, segundos=30.0, objetivo=1.0,
                 smc=SMC):
    """Lazo de altura a 50 Hz sobre un punto de masa. Devuelve (z, control)."""
    ctl = ControlAlturaSMC(PAR, smc)
    ctl.objetivo = objetivo
    z, vz, empuje, dt = 0.0, 0.0, masa_real * G, 0.02
    ctl.reset(z)
    for _ in range(int(segundos / dt)):
        mando = ctl.actualizar(z, vz, dt)
        empuje += dt / (dt + TAU_MOTOR) * (mando * EMPUJE_TOTAL - empuje)
        vz += (empuje + fuerza_externa - masa_real * G) / masa_real * dt
        z += vz * dt
    return z, ctl


def volar_roll(par_externo, smc=SMC, segundos=15.0):
    """Lazo de roll a 250 Hz con un par constante. Devuelve (roll real, velocidad).

    Reproduce lo que pasa en vuelo: el acelerómetro no ve la inclinación, así que el
    filtro complementario lleva el ángulo estimado a cero con su constante de tiempo.
    """
    ctl = ControlActitudSMC(PAR, smc)
    roll, p, roll_est, tau, dt = 0.0, 0.0, 0.0, 0.0, 0.004
    por_corr = 4.0 * smc.EMPUJE_MAX * smc.BRAZO
    for _ in range(int(segundos / dt)):
        ctl.actualizar(Est(roll=math.degrees(roll_est), vel_roll=math.degrees(p)), dt)
        tau += dt / (dt + TAU_MOTOR) * (ctl.corr_roll * por_corr - tau)
        p += (tau + par_externo) / smc.IXX * dt
        roll += p * dt
        roll_est += (p - roll_est / 0.5) * dt
    return roll, p


def test_sat_es_lineal_en_la_capa_limite_y_acotada_fuera():
    assert sat(0.1, 0.5) == pytest.approx(0.2)
    assert sat(5.0, 0.5) == 1.0
    assert sat(-5.0, 0.5) == -1.0


def test_ley_coincide_con_la_ecuacion_16():
    assert ley_smc(2.0, 1.0, 3.0, 0.5, 0.1) == pytest.approx(2.0 - 3.0 - 0.5)
    assert ley_smc(2.0, 0.0, 3.0, 0.5, 0.1) == pytest.approx(2.0)


def test_sobre_la_superficie_solo_queda_el_termino_equivalente():
    """En s = 0, con el dron quieto en la consigna, el mando es el de hover."""
    ctl = ControlAlturaSMC(PAR, SMC)
    ctl.reset(1.0)
    ctl.objetivo = 1.0
    assert ctl.actualizar(1.0, 0.0, 0.02) == pytest.approx(PAR.THROTTLE_HOVER)


def test_inclinado_pide_mas_empuje():
    ctl = ControlAlturaSMC(PAR, SMC)
    ctl.reset(1.0)
    ctl.objetivo = 1.0
    assert ctl.actualizar(1.0, 0.0, 0.02, 0.9) == pytest.approx(PAR.THROTTLE_HOVER / 0.9)


def test_por_debajo_de_la_consigna_sube():
    ctl = ControlAlturaSMC(PAR, SMC)
    ctl.reset(1.0)
    ctl.objetivo = 1.0
    assert ctl.actualizar(0.8, 0.0, 0.02) > PAR.THROTTLE_HOVER


def test_referencia_respeta_los_limites():
    ctl = ControlAlturaSMC(PAR, SMC)
    ctl.reset(0.0)
    ctl.objetivo = 3.0
    for _ in range(1000):
        ctl.actualizar(ctl.consigna, ctl.consigna_vel, 0.02)
        assert abs(ctl.consigna_vel) <= PAR.VEL_Z_MAX + 1e-9
        assert abs(ctl.consigna_acc) <= SMC.ACC_REF_MAX + 1e-9
    assert ctl.consigna == pytest.approx(3.0, abs=0.01)


def test_sin_adaptacion_el_error_cumple_la_cota_de_la_capa_limite():
    """Dentro de la capa límite una perturbación constante D deja
    s = D / (kd + rho/phi), así que e = s / lambda."""
    smc = sin_adaptacion()
    hover_real = smc.MASA * G / EMPUJE_TOTAL
    delta = (PAR.THROTTLE_HOVER - hover_real) * EMPUJE_TOTAL
    z, _ = volar_altura(smc=smc)
    cota = delta / (smc.KD_Z + smc.RHO_Z / smc.PHI_Z) / smc.LAMBDA_Z
    assert 0.5 * cota < abs(z - 1.0) <= cota * 1.1


def test_la_adaptacion_elimina_el_error_de_regimen():
    z, ctl = volar_altura()
    assert abs(z - 1.0) < 0.01
    assert ctl.sesgo < 0.0      # THROTTLE_HOVER pide de más


def test_sin_adaptacion_un_par_constante_tumba_al_dron():
    """El fallo que se vio en Gazebo: la capa límite deja s != 0 y, como el ángulo
    estimado cae a cero, eso es una velocidad de giro constante."""
    smc = sin_adaptacion()
    roll, p = volar_roll(0.10, smc)
    pendiente = smc.KD_ANG + smc.RHO_ANG / smc.PHI_ANG
    esperada = 0.10 / pendiente / (1.0 + smc.LAMBDA_ANG * 0.5)
    assert abs(p) == pytest.approx(esperada, rel=0.05)
    assert abs(math.degrees(roll)) > 30.0


def test_la_adaptacion_rechaza_un_par_constante():
    roll, p = volar_roll(0.10)
    assert abs(math.degrees(roll)) < 15.0
    assert abs(p) < 0.01


def test_adaptar_solo_dentro_de_la_capa_limite():
    assert adaptar(0.2, 1.0, 0.5, 3.0, 1.0, 0.02) == 0.2
    assert adaptar(0.2, 0.1, 0.5, 3.0, 1.0, 0.02) == pytest.approx(0.2 - 3.0 * 0.1 * 0.02)


def test_adaptar_acotado():
    assert adaptar(0.99, -0.4, 0.5, 100.0, 1.0, 0.1) == 1.0


def test_reset_borra_el_sesgo():
    ctl = ControlActitudSMC(PAR, SMC)
    for _ in range(100):
        ctl.actualizar(Est(vel_roll=5.0, vel_yaw=5.0), 0.004)
    assert ctl.sesgo != [0.0, 0.0, 0.0]
    ctl.reset()
    assert ctl.sesgo == [0.0, 0.0, 0.0]


@pytest.mark.parametrize('masa', [1.0, 1.15, 1.35])
def test_robusto_a_error_de_masa(masa):
    z, _ = volar_altura(masa_real=masa)
    assert abs(z - 1.0) < 0.25


def test_robusto_a_fuerza_externa():
    z, _ = volar_altura(fuerza_externa=-2.0)
    assert abs(z - 1.0) < 0.25


def test_signo_roll_positivo_corrige_hacia_abajo():
    """Inclinado a la derecha hay que devolverlo: corrección negativa."""
    ctl = ControlActitudSMC(PAR, SMC)
    ctl.actualizar(Est(roll=5.0), 0.004)
    assert ctl.corr_roll < 0.0


def test_signo_pitch_y_consigna():
    ctl = ControlActitudSMC(PAR, SMC)
    ctl.sp_pitch = 10.0
    ctl.actualizar(Est(), 0.004)
    assert ctl.corr_pitch > 0.0


def test_consigna_de_angulo_acotada():
    a, b = ControlActitudSMC(PAR, SMC), ControlActitudSMC(PAR, SMC)
    a.sp_roll, b.sp_roll = 500.0, PAR.ANG_SP_MAX
    a.actualizar(Est(), 0.004)
    b.actualizar(Est(), 0.004)
    assert a.corr_roll == b.corr_roll


def test_correcciones_acotadas():
    ctl = ControlActitudSMC(PAR, SMC)
    ctl.actualizar(Est(roll=90.0, pitch=-90.0, vel_yaw=500.0), 0.004)
    for corr in (ctl.corr_roll, ctl.corr_pitch, ctl.corr_yaw):
        assert abs(corr) <= PAR.CORR_MAX


def test_guinada_frena_el_giro():
    ctl = ControlActitudSMC(PAR, SMC)
    ctl.actualizar(Est(vel_yaw=30.0), 0.004)
    assert ctl.corr_yaw < 0.0
