"""El firmware y la simulación tienen que aplicar la misma ley.

Compila src_microros/src/smc.h en el PC y compara su salida con robot_control/smc.py.
Se salta si no está g++ o si el firmware no está junto al workspace.
"""

import math
import re
import shutil
import subprocess
from pathlib import Path

import pytest
from robot_control.control import G, ParametrosControl
from robot_control.smc import ControlActitudSMC, ControlAlturaSMC, ParametrosSMC

FIRMWARE = Path(__file__).resolve().parents[4] / 'src_microros' / 'src'
PAR = ParametrosControl()
SMC = ParametrosSMC()

pytestmark = pytest.mark.skipif(
    shutil.which('g++') is None or not (FIRMWARE / 'smc.h').exists(),
    reason='hace falta g++ y el firmware')

ARNES = r'''
#include <cstdio>
#include "smc.h"
int main() {
    float a, sp, r, g, sesgo = 0.0f;
    while (scanf("%%f %%f %%f %%f", &a, &sp, &r, &g) == 4) {
        EjeSMC e = smcEje(a, sp, r, %(IXX)sf, g, %(LAMBDA_ANG)sf, %(KD_ANG)sf,
                          %(RHO_ANG)sf, %(PHI_ANG)sf, %(P_ANG)sf, %(SESGO_MAX_ANG)sf,
                          &sesgo, 0.004f);
        printf("%%.7g %%.7g %%.7g\n", e.tau, e.s, e.velRef);
    }
    return 0;
}
'''

ARNES_Z = r'''
#include <cstdio>
#include "smc.h"
int main() {
    ReferenciaZ ref;
    ref.reset(0.0f);
    float z, vz, sesgo = 0.0f;
    while (scanf("%%f %%f", &z, &vz) == 2) {
        ref.actualizar(1.0f, %(OMEGA_REF_Z)sf, %(ACC_REF_MAX)sf, %(VEL)sf, 0.02f);
        float velD, s;
        float f = smcAltura(ref, z, vz, %(MASA)sf, %(G)sf, %(LAMBDA_Z)sf, %(KD_Z)sf,
                            %(RHO_Z)sf, %(PHI_Z)sf, %(P_Z)sf, %(SESGO_MAX_Z)sf,
                            &sesgo, 0.02f, &velD, &s);
        printf("%%.7g %%.7g %%.7g %%.7g\n", ref.z, velD, s, f);
    }
    return 0;
}
'''


def correr(codigo, tmp_path, entrada):
    fuente, binario = tmp_path / 'arnes.cpp', tmp_path / 'arnes'
    fuente.write_text(codigo)
    subprocess.run(['g++', '-std=c++11', '-I', str(FIRMWARE), str(fuente), '-o', str(binario)],
                   check=True)
    salida = subprocess.run([str(binario)], input=entrada, text=True, capture_output=True,
                            check=True).stdout
    return [[float(v) for v in linea.split()] for linea in salida.splitlines()]


def valores():
    return {k: repr(float(getattr(SMC, k))) for k in dir(SMC) if k.isupper()}


def test_constantes_iguales_en_config_h():
    texto = (FIRMWARE / 'config.h').read_text()
    definidas = dict(re.findall(r'#define\s+SMC_(\w+)\s+([-\d.]+)f', texto))
    constantes = [k for k in dir(SMC) if k.isupper()]
    assert sorted(definidas) == sorted(constantes)
    for nombre in constantes:
        assert float(definidas[nombre]) == pytest.approx(getattr(SMC, nombre)), nombre


def test_eje_de_actitud_igual_al_firmware(tmp_path):
    casos = [(0.0, 0.0, 0.0, 0.0), (0.1, 0.0, 0.0, 0.0), (-0.2, 0.1, 0.5, 0.01),
             (0.05, 0.05, -1.5, 0.0), (0.3, -0.26, 2.0, -0.02), (0.001, 0.0, 0.002, 0.0)]
    entrada = '\n'.join(' '.join(str(v) for v in c) for c in casos)
    firmware = correr(ARNES % valores(), tmp_path, entrada)
    ctl = ControlActitudSMC(PAR, SMC)
    assert len(firmware) == len(casos)
    for (a, sp, r, g), (tau, s, vel_ref) in zip(casos, firmware):
        esperado = ctl._eje(a, sp, r, SMC.IXX, g, 0, 0.004)
        assert esperado == pytest.approx((tau, s, vel_ref), rel=1e-4, abs=1e-5)


def test_altura_igual_al_firmware(tmp_path):
    casos = [(0.1 * math.sin(0.1 * k), 0.3 * math.cos(0.2 * k)) for k in range(200)]
    entrada = '\n'.join('%.6f %.6f' % c for c in casos)
    v = valores()
    v.update(VEL=repr(PAR.VEL_Z_MAX), G=repr(G))
    firmware = correr(ARNES_Z % v, tmp_path, entrada)

    ctl = ControlAlturaSMC(PAR, SMC)
    ctl.reset(0.0)
    ctl.objetivo = 1.0
    for (z, vz), (ref, vel_d, s, fuerza) in zip(casos, firmware):
        ctl.actualizar(round(z, 6), round(vz, 6), 0.02)
        assert ctl.consigna == pytest.approx(ref, rel=1e-3, abs=1e-4)
        assert ctl.vz_deseada == pytest.approx(vel_d, rel=1e-3, abs=1e-4)
        assert ctl.s == pytest.approx(s, rel=1e-3, abs=1e-4)
        acc = max(-PAR.ACC_Z_MAX, min(PAR.ACC_Z_MAX, fuerza / SMC.MASA - G))
        assert ctl.acc_deseada == pytest.approx(acc, rel=1e-3, abs=1e-3)
