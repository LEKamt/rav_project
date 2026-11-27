"""Genera un CSV como el de PrimerVueloWiFi_PIDv2 a partir de una planta CONOCIDA, para
comprobar que identificar.py recupera sus parametros. No reemplaza el ensayo real.

    python simular_ensayo.py sim.csv            # luego: python identificar.py sim.csv

Incluye lo que el modelo no tiene: ruido y vibracion en el giro, DLPF del giro (~4 ms),
friccion seca en el tubo y el angulo del filtro complementario.
"""
import sys
import numpy as np

VERDAD = dict(K=-30000.0, b=0.8, k=-3.0, c=40.0, tau=0.06, d_s=0.008)
fs, dur = 500, 20.0
kp, kd, ki = 0.003, 0.0006, 0.0
amp, f0, f1 = 0.03, 0.5, 10.0
sigma = 1.0          # eje roll
rng = np.random.default_rng(1)


def main(ruta):
    Ts = 1 / fs
    h = Ts / 4
    n = int(dur * fs)
    d = int(round(VERDAD["d_s"] * fs))
    al = np.exp(-h / VERDAD["tau"])
    a_dlpf = np.exp(-Ts / 0.004)
    th, w, uf, integ = 2.0, 0.0, 0.0, 0.0
    w_med, th_est = 0.0, 2.0
    buf_w = [0.0] * 5
    buf_u = [0.0] * (d + 1)
    fase, t = 0.0, 0.0
    filas = []
    for i in range(n):
        # sensores
        w_med = a_dlpf * w_med + (1 - a_dlpf) * (w + rng.normal(0, 3.0) + 2.0 * np.sin(2 * np.pi * 95 * t))
        th_acc = th + rng.normal(0, 1.5)
        al_c = 0.5 / (0.5 + Ts)
        th_est = al_c * (th_est + w_med * Ts) + (1 - al_c) * th_acc
        # controlador del firmware
        buf_w = buf_w[1:] + [w_med]
        f = f0 * (f1 / f0) ** (t / dur)
        fase += 2 * np.pi * f * Ts
        exc = amp * np.sin(fase)
        e = sigma * th_est
        u = kp * e + kd * sigma * np.mean(buf_w) + integ + exc
        integ += ki * e * Ts
        filas.append((int(Ts * 1e6), exc, u, th_est, w_med, 0.45, 0))
        buf_u = buf_u[1:] + [u]
        for _ in range(4):
            uf = al * uf + (1 - al) * buf_u[0]
            fric = -60.0 * np.tanh(w / 2.0)       # friccion seca del tubo
            w += h * (VERDAD["K"] * uf - VERDAD["b"] * w - VERDAD["k"] * th + VERDAD["c"] + fric)
            th += h * w
        t += Ts
    with open(ruta, "w", encoding="utf-8") as fo:
        fo.write(f"# eje=r fs={fs} kp={kp} kd={kd} ki={ki} db=0 thr=0.45\n")
        fo.write(f"# amp={amp} f0={f0} f1={f1} dur={dur} fin=completo\n")
        fo.write("dt_us,exc,u,ang,rate,thr,sat\n")
        for r in filas:
            fo.write("%d,%.4f,%.4f,%.2f,%.1f,%.3f,%d\n" % r)
    print(f"{ruta}: {n} muestras. Verdad: {VERDAD}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "sim.csv")
