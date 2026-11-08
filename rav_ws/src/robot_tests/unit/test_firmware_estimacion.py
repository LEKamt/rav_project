"""Filtros, AHRS y Kalman de altura del firmware, compilados en el PC.

Compila filtros.h, ahrs.h y kalman_altura.h de src_microros/src con g++ y los
ejercita con señales sintéticas. Se salta si no está g++.
"""

import math
import random
import shutil
import subprocess
from pathlib import Path

import pytest

FIRMWARE = Path(__file__).resolve().parents[4] / 'src_microros' / 'src'

pytestmark = pytest.mark.skipif(
    shutil.which('g++') is None or not (FIRMWARE / 'kalman_altura.h').exists(),
    reason='hace falta g++ y el firmware')

ARNES = r'''
#include <cstdio>
#include <cstring>
#include "filtros.h"
#include "ahrs.h"
#include "kalman_altura.h"

// Lee órdenes por stdin, una por línea, y responde por stdout.
int main() {
    char cmd[16];
    Biquad bq;
    FiltroEjes fe;
    Mahony m;
    KalmanAltura k;
    while (scanf("%15s", cmd) == 1) {
        if (!strcmp(cmd, "lpf")) {            // fs fc
            float fs, fc; scanf("%f %f", &fs, &fc); bq.pasabajos(fs, fc);
        } else if (!strcmp(cmd, "notch")) {   // fs f0 bw
            float fs, f0, bw; scanf("%f %f %f", &fs, &f0, &bw); bq.notch(fs, f0, bw);
        } else if (!strcmp(cmd, "breset")) {
            float x; scanf("%f", &x); bq.reset(x);
        } else if (!strcmp(cmd, "b")) {       // un paso del biquad
            float x; scanf("%f", &x); printf("%.9g\n", bq.paso(x));
        } else if (!strcmp(cmd, "mcfg")) {
            float kp, ki, rj; scanf("%f %f %f", &kp, &ki, &rj); m.configurar(kp, ki, rj);
        } else if (!strcmp(cmd, "mini")) {
            float a[3]; scanf("%f %f %f", &a[0], &a[1], &a[2]); m.iniciarConAcc(a);
        } else if (!strcmp(cmd, "m")) {       // gx gy gz ax ay az dt
            float g[3], a[3], dt;
            scanf("%f %f %f %f %f %f %f", &g[0], &g[1], &g[2], &a[0], &a[1], &a[2], &dt);
            m.actualizar(g, a, dt);
        } else if (!strcmp(cmd, "me")) {
            float r, p, y; m.euler(&r, &p, &y);
            printf("%.9g %.9g %.9g %.9g\n", r, p, y, m.cosInclinacion());
        } else if (!strcmp(cmd, "maz")) {     // ax ay az -> acc vertical
            float a[3]; scanf("%f %f %f", &a[0], &a[1], &a[2]);
            printf("%.9g\n", m.aceleracionVertical(a, 9.80665f));
        } else if (!strcmp(cmd, "kreset")) {
            float z, b; scanf("%f %f", &z, &b); k.reset(z, b);
        } else if (!strcmp(cmd, "kp")) {      // acc dt
            float a, dt; scanf("%f %f", &a, &dt); k.predecir(a, dt);
        } else if (!strcmp(cmd, "kd")) {      // sensor medida sigma
            int s; float z, sg; scanf("%d %f %f", &s, &z, &sg);
            printf("%d\n", (int)k.corregirDistancia(s, z, sg));
        } else if (!strcmp(cmd, "kb")) {      // medida sigma
            float z, sg; scanf("%f %f", &z, &sg);
            printf("%d\n", (int)k.corregirBaro(z, sg));
        } else if (!strcmp(cmd, "ke")) {
            printf("%.9g %.9g %.9g %.9g\n", k.z, k.vz, k.b, k.P[0][0]);
        }
        fflush(stdout);
    }
    return 0;
}
'''


@pytest.fixture(scope='module')
def binario(tmp_path_factory):
    d = tmp_path_factory.mktemp('estimacion')
    fuente = d / 'arnes.cpp'
    fuente.write_text(ARNES)
    exe = d / 'arnes'
    subprocess.run(['g++', '-O2', '-Wall', '-Wextra', '-Werror', '-Wno-unused-result',
                    '-I', str(FIRMWARE), str(fuente), '-o', str(exe)],
                   check=True, capture_output=True, text=True)
    return exe


def correr(binario, lineas):
    """Corre las órdenes y devuelve las líneas de salida."""
    r = subprocess.run([str(binario)], input='\n'.join(lineas) + '\n',
                       capture_output=True, text=True, check=True)
    return r.stdout.split('\n')[:-1]


# ===================== filtros =====================

def amplitud(binario, cfg, fs, f, n=4000):
    """Amplitud de régimen de una senoidal de frecuencia f tras el biquad."""
    ordenes = [cfg]
    for i in range(n):
        ordenes.append('b %.9g' % math.sin(2 * math.pi * f * i / fs))
    y = [float(v) for v in correr(binario, ordenes)]
    return max(abs(v) for v in y[n // 2:])


def respuesta_lpf(fs, fc, f):
    """|H| teórica del Butterworth digital del Cookbook (bilineal, cero en fs/2)."""
    w0 = 2 * math.pi * fc / fs
    alpha = math.sin(w0) / (2 * 0.70710678)
    cw = math.cos(w0)
    b = [(1 - cw) / 2, 1 - cw, (1 - cw) / 2]
    a = [1 + alpha, -2 * cw, 1 - alpha]
    z = complex(math.cos(2 * math.pi * f / fs), math.sin(2 * math.pi * f / fs))
    h = (b[0] + b[1] / z + b[2] / z ** 2) / (a[0] + a[1] / z + a[2] / z ** 2)
    return abs(h)


@pytest.mark.parametrize('f', [5.0, 30.0, 60.0, 120.0, 200.0])
def test_pasabajos_butterworth(binario, f):
    a = amplitud(binario, 'lpf 500 60', 500.0, f)
    assert a == pytest.approx(respuesta_lpf(500.0, 60.0, f), abs=0.01)
    if f == 60.0:
        assert a == pytest.approx(0.707, abs=0.01)   # -3 dB en el corte


def test_notch_elimina_su_frecuencia_y_deja_el_resto(binario):
    assert amplitud(binario, 'notch 500 90 30', 500.0, 90.0) < 0.02
    assert amplitud(binario, 'notch 500 90 30', 500.0, 10.0) > 0.97


def test_frecuencia_invalida_deja_paso_directo(binario):
    # Por encima de Nyquist no debe quedar un filtro inestable
    y = correr(binario, ['lpf 500 300', 'b 0.5', 'b -2'])
    assert [float(v) for v in y] == [0.5, -2.0]


def test_reset_arranca_en_regimen(binario):
    y = correr(binario, ['lpf 500 60', 'breset 3', 'b 3', 'b 3'])
    assert all(float(v) == pytest.approx(3.0, abs=1e-5) for v in y)


# ===================== AHRS =====================

def acc_reposo(roll, pitch):
    """Fuerza específica [g] en FLU con el dron quieto e inclinado."""
    return (-math.sin(pitch), math.cos(pitch) * math.sin(roll),
            math.cos(pitch) * math.cos(roll))


def test_convencion_de_signos_de_la_mezcla(binario):
    """Lado izquierdo arriba = alabeo positivo; nariz abajo = cabeceo positivo.
    Es lo que asume la mezcla de motores.cpp."""
    r, p = math.radians(12), math.radians(-7)
    ax, ay, az = acc_reposo(r, p)
    sal = correr(binario, ['mini %f %f %f' % (ax, ay, az), 'me'])
    roll, pitch, yaw, c = map(float, sal[0].split())
    assert roll == pytest.approx(r, abs=1e-4)
    assert pitch == pytest.approx(p, abs=1e-4)
    assert yaw == pytest.approx(0.0, abs=1e-6)
    assert c == pytest.approx(math.cos(r) * math.cos(p), abs=1e-4)


def test_converge_a_la_gravedad(binario):
    ax, ay, az = acc_reposo(math.radians(10), math.radians(5))
    dt = 0.002
    ordenes = ['mcfg 1.0 0.0 0.15', 'mini 0 0 1']
    ordenes += ['m 0 0 0 %f %f %f %f' % (ax, ay, az, dt)] * int(8.0 / dt)
    ordenes.append('me')
    roll, pitch, _, _ = map(float, correr(binario, ordenes)[-1].split())
    assert math.degrees(roll) == pytest.approx(10.0, abs=0.2)
    assert math.degrees(pitch) == pytest.approx(5.0, abs=0.2)


def test_giro_integra_y_estima_el_sesgo(binario):
    """Con un sesgo residual de 1 deg/s en x, el término integral lo compensa
    y el alabeo se queda en cero."""
    dt = 0.002
    sesgo = math.radians(1.0)
    ordenes = ['mcfg 1.0 0.05 0.15', 'mini 0 0 1']
    ordenes += ['m %f 0 0 0 0 1 %f' % (sesgo, dt)] * int(60.0 / dt)
    ordenes.append('me')
    roll = float(correr(binario, ordenes)[-1].split()[0])
    assert abs(math.degrees(roll)) < 0.1


def test_rechaza_aceleraciones_de_maniobra(binario):
    """Un golpe de 2 g no tumba la estimación."""
    dt = 0.002
    ordenes = ['mcfg 1.0 0.0 0.15', 'mini 0 0 1']
    ordenes += ['m 0 0 0 0 2.0 0.3 %f' % dt] * 200
    ordenes.append('me')
    roll = float(correr(binario, ordenes)[-1].split()[0])
    assert abs(roll) < 1e-6


def test_aceleracion_vertical_sin_gravedad(binario):
    ax, ay, az = acc_reposo(math.radians(15), math.radians(-10))
    sal = correr(binario, ['mini %f %f %f' % (ax, ay, az),
                           'maz %f %f %f' % (ax, ay, az),
                           'maz %f %f %f' % (1.2 * ax, 1.2 * ay, 1.2 * az)])
    assert float(sal[0]) == pytest.approx(0.0, abs=1e-4)
    assert float(sal[1]) == pytest.approx(0.2 * 9.80665, abs=1e-3)


# ===================== Kalman de altura =====================

def vuelo(binario, semilla=1, tof=True, sonar=True, baro=True, ecos=0.0):
    """Sube de 0 a 2.5 m, se queda y baja, con los sensores del F450. Devuelve el
    error RMS de altura, el de velocidad y el error final."""
    rnd = random.Random(semilla)
    dt = 0.02                    # tarea lenta, 50 Hz
    deriva_baro = 0.0
    ordenes = ['kreset 0 0']
    verdad = []
    t = 0.0
    z = vz = 0.0
    for i in range(int(30.0 / dt)):
        # Perfil: acelera hasta 0.5 m/s, sube, se detiene, baja
        if t < 1.0:
            a = 0.5
        elif t < 5.0:
            a = 0.0
        elif t < 6.0:
            a = -0.5
        elif t < 18.0:
            a = 0.0
        elif t < 19.0:
            a = -0.5
        elif t < 23.0:
            a = 0.0
        elif t < 24.0:
            a = 0.5
        else:
            a = 0.0
        vz += a * dt
        z = max(0.0, z + vz * dt)
        t += dt
        deriva_baro += 0.01 * dt          # 1 cm/s de deriva del baro
        ordenes.append('kp %f %f' % (a + rnd.gauss(0, 0.3), dt))
        if tof and z < 1.0:
            ordenes.append('kd 0 %f 0.02' % (z + rnd.gauss(0, 0.01)))
        if sonar and i % 3 == 0 and 0.05 < z < 4.0:
            medida = z + rnd.gauss(0, 0.03)
            if rnd.random() < ecos:
                medida += rnd.choice([-1, 1]) * rnd.uniform(0.5, 2.0)   # eco falso
            ordenes.append('kd 1 %f 0.05' % medida)
        if baro and i % 2 == 0:
            ordenes.append('kb %f 0.35' % (z + deriva_baro + rnd.gauss(0, 0.25)))
        ordenes.append('ke')
        verdad.append((z, vz))
    est = [tuple(map(float, l.split())) for l in correr(binario, ordenes)
           if len(l.split()) == 4]
    ez = [e[0] - v[0] for e, v in zip(est, verdad)]
    ev = [e[1] - v[1] for e, v in zip(est, verdad)]
    rms = lambda xs: math.sqrt(sum(x * x for x in xs) / len(xs))
    return rms(ez), rms(ev), abs(ez[-1])


def test_kalman_con_los_tres_sensores(binario):
    ez, ev, _ = vuelo(binario)
    assert ez < 0.04
    assert ev < 0.10


def test_kalman_sin_tof_sigue_con_sonar(binario):
    ez, _, _ = vuelo(binario, tof=False)
    assert ez < 0.06


def test_kalman_solo_baro_deriva_acotado(binario):
    """Solo con el baro no hay referencia absoluta, pero el error no explota."""
    ez, _, _ = vuelo(binario, tof=False, sonar=False)
    assert ez < 0.5


def test_kalman_rechaza_ecos_falsos_del_sonar(binario):
    limpio, _, _ = vuelo(binario, tof=False, semilla=3)
    sucio, _, _ = vuelo(binario, tof=False, semilla=3, ecos=0.15)
    assert sucio < 0.08
    assert sucio < 2.0 * limpio + 0.02


def test_kalman_estima_el_sesgo_del_baro(binario):
    """Con el dron quieto en 0.5 m y el baro corrido 1.3 m, el sesgo se aprende
    del ToF y la altura no se va al baro."""
    ordenes = ['kreset 0.5 0']
    for i in range(500):
        ordenes.append('kp 0 0.02')
        ordenes.append('kd 0 0.5 0.02')
        if i % 2 == 0:
            ordenes.append('kb 1.8 0.35')
    ordenes.append('ke')
    z, vz, b, _ = map(float, correr(binario, ordenes)[-1].split())
    assert z == pytest.approx(0.5, abs=0.02)
    assert b == pytest.approx(1.3, abs=0.05)


def test_kalman_reancla_si_el_sensor_insiste(binario):
    """Si el filtro se perdió (solo baro mucho rato) y vuelve el ToF con otra
    altura, tras unos rechazos acepta la medida."""
    ordenes = ['kreset 0 0']
    ordenes += ['kp 0 0.02'] * 10
    ordenes += ['kd 0 0.8 0.02'] * 20
    ordenes.append('ke')
    sal = correr(binario, ordenes)
    aceptadas = [int(v) for v in sal[:-1]]
    assert aceptadas[0] == 0                 # el primer salto se rechaza
    assert 1 in aceptadas                    # pero termina entrando
    z = float(sal[-1].split()[0])
    assert z == pytest.approx(0.8, abs=0.02)
