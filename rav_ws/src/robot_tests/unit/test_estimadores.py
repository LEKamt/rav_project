"""Pruebas de la estimación de actitud y de altura (robot_estimation)."""

import math

from robot_estimation.estimacion import (EstimadorActitud, EstimadorAltura,
                                          ParametrosEstimacion)

PAR = ParametrosEstimacion()
DT = 1.0 / 250.0


def quieto_inclinado(roll_deg):
    """Medida del acelerómetro de un dron quieto con cierto alabeo."""
    r = math.radians(roll_deg)
    return (0.0, math.sin(r), math.cos(r))


def test_el_filtro_converge_al_angulo_del_acelerometro():
    est = EstimadorActitud(PAR)
    acc = quieto_inclinado(10.0)
    for _ in range(int(5.0 / DT)):          # 5 s, 10 constantes de tiempo
        est.actualizar(acc, (0.0, 0.0, 0.0), DT)
    assert abs(est.roll - 10.0) < 0.5


def test_el_giroscopo_manda_a_corto_plazo():
    """Una rotación rápida la ve el giróscopo antes que el acelerómetro: es el
    sentido del filtro complementario."""
    est = EstimadorActitud(PAR)
    for _ in range(int(0.2 / DT)):          # 0.2 s a 50 deg/s
        est.actualizar((0.0, 0.0, 1.0), (50.0, 0.0, 0.0), DT)
    assert 7.0 < est.roll < 10.0            # ~10 deg menos lo que tira el acc


def test_velocidad_angular_filtrada_sigue_al_giroscopo():
    est = EstimadorActitud(PAR)
    for _ in range(int(0.5 / DT)):
        est.actualizar((0.0, 0.0, 1.0), (0.0, 30.0, 0.0), DT)
    assert abs(est.vel_pitch - 30.0) < 1.0


def test_aceleracion_vertical_cero_en_hover():
    est = EstimadorActitud(PAR)
    for _ in range(int(2.0 / DT)):
        est.actualizar((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), DT)
    assert abs(est.acc_vertical) < 0.05


def test_aceleracion_vertical_detecta_el_empuje():
    """1.2 g medidos en Z con el dron nivelado son ~2 m/s^2 de ascenso."""
    est = EstimadorActitud(PAR)
    for _ in range(int(3.0 / DT)):
        est.actualizar((0.0, 0.0, 1.2), (0.0, 0.0, 0.0), DT)
    assert abs(est.acc_vertical - 0.2 * 9.80665) < 0.3


def test_el_yaw_se_envuelve():
    est = EstimadorActitud(PAR)
    for _ in range(int(4.0 / DT)):          # 4 s a 100 deg/s = 400 deg
        est.actualizar((0.0, 0.0, 1.0), (0.0, 0.0, 100.0), DT)
    assert -180.0 <= est.yaw <= 180.0


def test_cuaternion_normalizado():
    est = EstimadorActitud(PAR)
    est.roll, est.pitch, est.yaw = 10.0, -5.0, 30.0
    x, y, z, w = est.cuaternion()
    assert abs(math.sqrt(x * x + y * y + z * z + w * w) - 1.0) < 1e-9


# ---------- altura ----------

def test_por_debajo_del_metro_manda_el_lidar():
    est = EstimadorAltura(PAR)
    est.set_baro(5.0)           # barómetro disparatado a propósito
    est.set_lidar(0.5, True)
    for _ in range(100):
        est.actualizar(0.02)
    assert abs(est.altura - 0.5) < 1e-6


def test_sin_lidar_manda_el_barometro():
    est = EstimadorAltura(PAR)
    est.set_baro(2.0)
    est.set_lidar(0.0, False)
    est.actualizar(0.02)
    assert abs(est.altura - 2.0) < 1e-6


def test_el_anclaje_corrige_la_deriva_del_barometro():
    """Con el LiDAR válido y el barómetro desviado 0.3 m, el anclaje debe
    aprender ese sesgo; al perder el LiDAR, la altura no debe saltar."""
    est = EstimadorAltura(PAR)
    for _ in range(400):                    # 8 s a 50 Hz
        est.set_lidar(0.50, True)
        est.set_baro(0.80)                  # 0.3 m de más
        est.actualizar(0.02)
    altura_con_lidar = est.altura
    est.set_lidar(0.0, False)               # se pierde el LiDAR
    est.actualizar(0.02)
    assert abs(est.altura - altura_con_lidar) < 0.05
    assert abs(est.correccion_baro + 0.30) < 0.05


def test_velocidad_vertical_sigue_a_la_rampa():
    est = EstimadorAltura(PAR)
    z = 0.0
    for _ in range(int(3.0 / 0.02)):        # 3 s subiendo a 0.3 m/s
        z += 0.3 * 0.02
        est.set_lidar(z, True)
        est.actualizar(0.02)
    assert abs(est.vz - 0.3) < 0.05


def test_velocidad_vertical_acotada_ante_un_salto():
    """Un salto de la fuente no debe meter un pico de varios m/s en el término
    derivativo del PID."""
    est = EstimadorAltura(PAR)
    est.set_lidar(0.2, True)
    est.actualizar(0.02)
    est.set_lidar(1.4, True)                # salto de 1.2 m en un ciclo
    est.actualizar(0.02)
    assert abs(est.vz) <= PAR.VZ_MAX_CRUDA + 1e-9
