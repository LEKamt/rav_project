"""El sketch de Arduino (hardware/VueloF450) y el firmware micro-ROS comparten las leyes.

- Las cabeceras de hardware/VueloF450/src son copias de src_microros/src: tienen que ser iguales.
- mezcla.h aplica la misma mezcla que robot_control/control.py::mezclar.
- Los signos de la mezcla corresponden a los sentidos de giro de hardware/CABLEADO_S3.md.
"""

import random
import shutil
import subprocess
from pathlib import Path

import pytest
from robot_control.control import ParametrosControl, mezclar

RAIZ = Path(__file__).resolve().parents[4]
FIRMWARE = RAIZ / 'src_microros' / 'src'
SKETCH = RAIZ / 'hardware' / 'VueloF450'
COMPARTIDAS = ('filtros.h', 'ahrs.h', 'smc.h', 'kalman_altura.h', 'mezcla.h')


@pytest.mark.parametrize('nombre', COMPARTIDAS)
def test_cabeceras_del_sketch_iguales_al_firmware(nombre):
    original = (FIRMWARE / nombre).read_bytes().replace(b'\r\n', b'\n')
    copia = (SKETCH / 'src' / nombre).read_bytes().replace(b'\r\n', b'\n')
    assert copia == original, f'hardware/VueloF450/src/{nombre} difiere de src_microros/src/{nombre}'


def test_el_sketch_incluye_las_cabeceras_compartidas():
    ino = (SKETCH / 'VueloF450.ino').read_text(encoding='utf-8')
    for nombre in COMPARTIDAS:
        assert f'#include "src/{nombre}"' in ino


ARNES = r'''
#include <cstdio>
#include "mezcla.h"
int main() {
    float t, r, p, y, lo, hi, m[4];
    while (scanf("%f %f %f %f %f %f", &t, &r, &p, &y, &lo, &hi) == 6) {
        mezclarX(t, r, p, y, lo, hi, m);
        printf("%.7g %.7g %.7g %.7g\n", m[0], m[1], m[2], m[3]);
    }
    return 0;
}
'''


@pytest.fixture(scope='module')
def mezcla_c(tmp_path_factory):
    if shutil.which('g++') is None:
        pytest.skip('hace falta g++')
    d = tmp_path_factory.mktemp('mezcla')
    (d / 'arnes.cpp').write_text(ARNES)
    exe = d / 'arnes'
    subprocess.run(['g++', '-O2', '-Wall', '-Werror', '-Wno-unused-result', '-I', str(FIRMWARE),
                    str(d / 'arnes.cpp'), '-o', str(exe)], check=True, capture_output=True)

    def correr(casos):
        entrada = '\n'.join(' '.join(f'{v:.7g}' for v in c) for c in casos) + '\n'
        sal = subprocess.run([str(exe)], input=entrada, capture_output=True, text=True, check=True)
        return [list(map(float, l.split())) for l in sal.stdout.splitlines()]
    return correr


def test_mezcla_c_igual_a_python(mezcla_c):
    par = ParametrosControl()
    rnd = random.Random(4)
    casos = [(rnd.uniform(0, 1), rnd.uniform(-0.4, 0.4), rnd.uniform(-0.4, 0.4),
              rnd.uniform(-0.4, 0.4), par.MOTOR_IDLE, 1.0) for _ in range(300)]
    for caso, m_c in zip(casos, mezcla_c(casos)):
        m_py = mezclar(caso[0], caso[1], caso[2], caso[3], par)
        assert m_c == pytest.approx(m_py, abs=1e-5)


def test_signos_de_la_mezcla_con_los_sentidos_reales(mezcla_c):
    """M1 frente-izq y M3 tras-der horarias; FLU: alabeo + = izquierda arriba,
    cabeceo + = nariz abajo, guiñada + = antihorario (lo dan las horarias)."""
    base, roll, pitch, yaw = mezcla_c([(0.5, 0, 0, 0, 0.08, 1.0), (0.5, 0.1, 0, 0, 0.08, 1.0),
                                       (0.5, 0, 0.1, 0, 0.08, 1.0), (0.5, 0, 0, 0.1, 0.08, 1.0)])
    izquierda, traseros, horarias = (0, 3), (2, 3), (0, 2)
    for i in range(4):
        assert (roll[i] > base[i]) == (i in izquierda)
        assert (pitch[i] > base[i]) == (i in traseros)
        assert (yaw[i] > base[i]) == (i in horarias)


def test_tope_de_mando_conserva_el_par(mezcla_c):
    """Con el tope de las primeras pruebas (0.80) la diferencia entre motores se
    conserva desplazando el empuje común."""
    (m,) = mezcla_c([(0.75, 0.1, 0, 0, 0.08, 0.80)])
    assert max(m) == pytest.approx(0.80)
    assert m[0] - m[1] == pytest.approx(0.2, abs=1e-6)
