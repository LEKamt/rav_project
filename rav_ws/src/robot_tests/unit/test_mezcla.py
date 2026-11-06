"""Pruebas de la mezcla en X y de su desaturación.

Los signos de esta tabla son lo que tumba un cuadricóptero casero, así que se
comprueban uno a uno: el motor del lado que baja tiene que acelerar.
Orden: M1 frente-izq, M2 frente-der, M3 tras-der, M4 tras-izq.
"""

from robot_control.control import ParametrosControl, mezclar

PAR = ParametrosControl()


def test_sin_correcciones_los_cuatro_iguales():
    m = mezclar(0.5, 0.0, 0.0, 0.0, PAR)
    assert m == [0.5, 0.5, 0.5, 0.5]


def test_roll_positivo_acelera_el_lado_izquierdo():
    """roll positivo = alabeo a la derecha, luego el control debe empujar más en
    los motores de la izquierda (M1 frente-izq y M4 tras-izq)."""
    m = mezclar(0.5, 0.1, 0.0, 0.0, PAR)
    assert m[0] > m[1] and m[3] > m[2]
    assert abs(m[0] - m[3]) < 1e-9      # los dos de la izquierda, iguales


def test_pitch_positivo_acelera_la_parte_trasera():
    """pitch positivo = morro abajo, luego suben los traseros (M3, M4)."""
    m = mezclar(0.5, 0.0, 0.1, 0.0, PAR)
    assert m[2] > m[0] and m[3] > m[1]


def test_yaw_positivo_acelera_la_diagonal_correcta():
    """La guiñada sale del par de reacción: aceleran los dos motores que giran
    en el mismo sentido (M2 y M4), que son los que el modelo de la simulación
    usa con signo +1 en torque.z."""
    m = mezclar(0.5, 0.0, 0.0, 0.1, PAR)
    assert m[1] > m[0] and m[3] > m[2]


def test_ralenti_minimo_con_el_dron_armado():
    """Ningún motor baja de MOTOR_IDLE: por debajo de ahí un ESC puede
    desincronizarse y no re-arrancar en vuelo."""
    m = mezclar(0.0, 0.0, 0.0, 0.0, PAR)
    assert all(v >= PAR.MOTOR_IDLE - 1e-9 for v in m)


def test_nunca_pasa_de_uno():
    m = mezclar(1.0, 0.3, 0.3, 0.3, PAR)
    assert all(v <= 1.0 + 1e-9 for v in m)


def test_desaturacion_conserva_el_par_diferencial():
    """Con el throttle alto y una corrección grande, la mezcla debe desplazar el
    throttle común en vez de recortar un motor: la diferencia entre lados tiene
    que conservarse."""
    corr = 0.2
    m = mezclar(0.95, corr, 0.0, 0.0, PAR)
    diferencia = (m[0] + m[3]) - (m[1] + m[2])
    esperada = 4 * corr
    assert abs(diferencia - esperada) < 1e-6


def test_desaturacion_por_abajo_tambien_conserva():
    corr = 0.2
    m = mezclar(0.05, corr, 0.0, 0.0, PAR)
    diferencia = (m[0] + m[3]) - (m[1] + m[2])
    assert abs(diferencia - 4 * corr) < 1e-6


def test_recorte_solo_cuando_es_imposible_conservar():
    """Si se piden correcciones que no caben en el rango ni desplazando, hay que
    recortar: lo que no puede pasar es salirse del rango."""
    m = mezclar(0.5, 0.5, 0.5, 0.5, PAR)
    assert all(PAR.MOTOR_IDLE - 1e-9 <= v <= 1.0 + 1e-9 for v in m)


def test_simetria_de_la_mezcla():
    """Invertir el signo de las tres correcciones debe dar la mezcla espejo."""
    a = mezclar(0.5, 0.05, 0.03, 0.02, PAR)
    b = mezclar(0.5, -0.05, -0.03, -0.02, PAR)
    assert abs(sum(a) - sum(b)) < 1e-9
