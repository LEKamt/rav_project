"""Prueba de integración: levanta la simulación y corre un escenario completo.

Es lenta (cada escenario arranca Gazebo), así que está marcada y fuera de la
batería por defecto:

    pytest src/dron_tests/integration -q -s                 # todos
    pytest src/dron_tests/integration -q -s -k hover        # solo uno

Requiere el workspace sourceado. Usa mallas:=false: la física es idéntica y
evita que Gazebo parsee los 57 MB del chasis en cada arranque.
"""

import os
import signal
import subprocess
import time

import pytest

ESCENARIOS = ['hover_1m', 'escalon', 'failsafe_timeout']
ESPERA_GAZEBO = 15.0        # s hasta que hay telemetría
MARGEN = 60.0               # s de margen sobre la duración del escenario


def _lanzar(cmd):
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, start_new_session=True)


def _matar(proc):
    if proc.poll() is None:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=10)


@pytest.mark.parametrize('escenario', ESCENARIOS)
def test_escenario(escenario):
    sim = _lanzar(['ros2', 'launch', 'dron_simulation', 'simulacion.launch.py',
                   'gui:=false', 'mallas:=false'])
    try:
        time.sleep(ESPERA_GAZEBO)
        assert sim.poll() is None, 'la simulación se cayó al arrancar'

        corredor = subprocess.run(
            ['ros2', 'run', 'dron_tests', 'corredor', '--ros-args',
             '-p', 'escenario:=%s' % escenario],
            capture_output=True, text=True, timeout=ESPERA_GAZEBO + MARGEN + 40)
        print(corredor.stdout[-4000:])
        assert corredor.returncode == 0, 'el escenario %s no cumple los criterios' % escenario
    finally:
        _matar(sim)
        time.sleep(3)        # que gzserver suelte el puerto antes del siguiente
