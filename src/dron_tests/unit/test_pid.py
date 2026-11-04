"""Pruebas del PID y de la cascada de altura.

No necesitan ROS ni Gazebo: son las leyes de control puras.

    colcon test --packages-select dron_tests
    # o directamente:
    pytest src/dron_tests/unit -q
"""

import math

from dron_control.control import G, PID, ControlAltura, ParametrosControl


def simular(pid, consigna, x0, dx, dt, pasos):
    """Aplica el PID sobre una medida constante; devuelve la última salida."""
    salida = 0.0
    for _ in range(pasos):
        salida = pid.actualizar(consigna, x0, dx, dt)
    return salida


def test_proporcional_puro():
    pid = PID(kp=2.0, tau=0.0)
    # tau=0: la consigna entra sin filtrar, p = 1.0 -> salida 2.0
    assert abs(pid.actualizar(1.0, 0.0, 0.0, 0.01) - 2.0) < 1e-9


def test_filtro_de_consigna_no_salta():
    """Con tau > 0 la consigna llega suave, como en el PIDController del
    Lab_rav_03: en el primer paso no se ve el escalón completo."""
    pid = PID(kp=1.0, tau=1.0)
    pid.actualizar(1.0, 0.0, 0.0, 0.02)
    assert 0.0 < pid.consigna < 0.1
    # al cabo de varias constantes de tiempo, prácticamente llega
    for _ in range(500):
        pid.actualizar(1.0, 0.0, 0.0, 0.02)
    assert pid.consigna > 0.99


def test_derivada_sobre_la_medida():
    """El término D usa la derivada medida (dx), no la del error: con la medida
    subiendo, D frena aunque el error sea el mismo."""
    base = PID(kp=1.0, kd=1.0, tau=0.0)
    con_dx = PID(kp=1.0, kd=1.0, tau=0.0)
    s_base = base.actualizar(1.0, 0.0, 0.0, 0.01)
    s_dx = con_dx.actualizar(1.0, 0.0, 2.0, 0.01)
    assert s_dx < s_base


def test_integrador_acotado():
    """i_max limita el integrador: es la diferencia frente al PID del
    laboratorio, que no tiene anti-windup."""
    pid = PID(ki=10.0, tau=0.0, i_max=0.5)
    simular(pid, 1.0, 0.0, 0.0, 0.01, 1000)
    assert abs(pid.i) <= 0.5 + 1e-9


def test_salida_acotada_y_sin_windup():
    """Con la salida saturada y el error empujando hacia la saturación, el
    integrador no debe seguir creciendo."""
    pid = PID(kp=1.0, ki=5.0, tau=0.0, salida_max=1.0, i_max=10.0)
    simular(pid, 5.0, 0.0, 0.0, 0.01, 200)
    i_tras_saturar = pid.i
    simular(pid, 5.0, 0.0, 0.0, 0.01, 200)
    assert abs(pid.i - i_tras_saturar) < 1e-6
    assert pid.salida <= 1.0 + 1e-9


def test_con_la_salida_saturada_el_integrador_ni_arranca():
    """Si el término P ya satura la salida por sí solo, la integración
    condicional no deja que el integrador acumule nada: ni un paso. Esto es lo
    que evita que el dron se quede con throttle de más tras un transitorio
    largo."""
    pid = PID(kp=1.0, ki=5.0, tau=0.0, salida_max=1.0, i_max=10.0)
    simular(pid, 5.0, 0.0, 0.0, 0.01, 200)
    assert pid.i == 0.0
    assert abs(pid.salida - 1.0) < 1e-9


def test_integrador_descarga_al_cambiar_el_signo():
    """Con error pequeño (la salida no satura) el integrador acumula, y al
    invertirse el error se descarga y cambia de signo."""
    pid = PID(kp=0.1, ki=1.0, tau=0.0, salida_max=1.0, i_max=10.0)
    simular(pid, 0.2, 0.0, 0.0, 0.01, 200)
    assert pid.i > 0.0
    simular(pid, -0.2, 0.0, 0.0, 0.01, 400)
    assert pid.i < 0.0


def test_hover_sale_el_throttle_de_hover():
    """Quieto en la altura objetivo, la cascada debe pedir exactamente el
    throttle de hover: es la propiedad del feedforward de gravedad."""
    par = ParametrosControl()
    ctrl = ControlAltura(par)
    ctrl.objetivo = 1.0
    ctrl.reset(1.0)
    for _ in range(200):
        thr = ctrl.actualizar(1.0, 0.0, 0.0, 0.02)
    assert abs(thr - par.THROTTLE_HOVER) < 1e-3


def test_por_debajo_del_objetivo_sube_el_throttle():
    par = ParametrosControl()
    ctrl = ControlAltura(par)
    ctrl.objetivo = 1.5
    ctrl.reset(1.0)
    thr = ctrl.actualizar(1.0, 0.0, 0.0, 0.02)
    assert thr > par.THROTTLE_HOVER


def test_velocidad_vertical_acotada():
    """El lazo externo no puede pedir más de VEL_Z_MAX, pase lo que pase con el
    error de altura: es lo que limita la velocidad de ascenso."""
    par = ParametrosControl()
    ctrl = ControlAltura(par)
    ctrl.objetivo = 3.0
    ctrl.reset(0.0)
    for _ in range(300):
        ctrl.actualizar(0.0, 0.0, 0.0, 0.02)
        assert ctrl.pos_z.salida <= par.VEL_Z_MAX + 1e-9


def test_aceleracion_acotada():
    par = ParametrosControl()
    ctrl = ControlAltura(par)
    ctrl.objetivo = 3.0
    ctrl.reset(0.0)
    for _ in range(300):
        ctrl.actualizar(0.0, 0.0, 0.0, 0.02)
        assert abs(ctrl.vel_z.salida) <= par.ACC_Z_MAX + 1e-9
    # y el throttle resultante nunca se sale de [0, 1]
    assert 0.0 <= ctrl.throttle_base <= 1.0


def test_equivalencia_con_la_formula_del_laboratorio():
    """throttle = hover*(1 + a/g) debe equivaler a force = masa*(a + g)
    normalizada por el empuje máximo, que es como lo hace el plugin del
    Lab_rav_03."""
    par = ParametrosControl()
    masa, empuje_max_total = 1.15, 4 * 8.3
    a = 2.0
    throttle = par.THROTTLE_HOVER * (1.0 + a / G)
    fuerza_equivalente = throttle * empuje_max_total
    fuerza_lab = masa * (a + G)
    # Coinciden salvo el margen que el firmware mete en THROTTLE_HOVER (0.36
    # frente al 0.34 teórico): un 6 %.
    assert abs(fuerza_equivalente - fuerza_lab) / fuerza_lab < 0.07


def test_sin_dt_no_cambia_nada():
    pid = PID(kp=1.0, ki=1.0, tau=0.0)
    pid.actualizar(1.0, 0.0, 0.0, 0.01)
    antes = (pid.i, pid.salida)
    pid.actualizar(1.0, 0.0, 0.0, 0.0)
    assert (pid.i, pid.salida) == antes


def test_valores_no_finitos_no_rompen():
    """Un NaN en la medida no debe propagarse como excepción; el nodo lo filtra
    antes, pero el PID no debe explotar."""
    pid = PID(kp=1.0, tau=0.0)
    salida = pid.actualizar(1.0, float('nan'), 0.0, 0.01)
    assert math.isnan(salida) or isinstance(salida, float)
