"""Identificacion de un eje del F450 en el balancin y diseno del PD (solo numpy + matplotlib).

Uso:
    python identificar.py ident.csv                 # identifica y disena
    python identificar.py ident.csv --validar otro.csv   # ademas valida con otro ensayo

El CSV sale de PrimerVueloWiFi_PIDv2 (boton DESCARGAR CSV). Modelo del eje en el tubo:

    th'' = K * uf(t - d) - b * th' - k * th + c
    uf'  = (u - uf) / tau           (motor + ESC como primer orden)

    th  : angulo del eje [grados]        u : mando diferencial del mezclador [fraccion]
    K   : aceleracion por unidad de mando [grados/s^2]   (sale negativo si el signo de la P
          del firmware es el correcto: u > 0 tiene que bajar el angulo)
    b   : amortiguamiento [1/s]          k : rigidez del tubo [1/s^2] (> 0 si el pivote esta
          por encima del CG, < 0 si esta por debajo: pendulo invertido)
    c   : desbalance (CG corrido) [grados/s^2]
    tau : constante de tiempo de motor+ESC [s]      d : retardo puro [muestras]

Metodo: regresion lineal de la aceleracion angular (derivada del giro filtrado) sobre
[uf, w, th, 1] para una grilla de (tau, d); se queda con la de menor residuo. Luego valida
simulando el modelo en ventanas de 1 s y simulando el lazo cerrado completo con las
ganancias que tenia el dron. Por ultimo busca Kp, Kd con margenes minimos y menor ITAE.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

N_MA = 5          # media movil del giro para la D (igual que el firmware)
KP_MAX, KD_MAX = 0.05, 0.005   # limites del firmware


# ----------------------------------------------------------------------------- datos
def leer_csv(ruta):
    meta = {}
    filas = []
    with open(ruta, encoding="utf-8") as f:
        for linea in f:
            linea = linea.strip()
            if not linea:
                continue
            if linea.startswith("#"):
                for par in linea[1:].split():
                    if "=" in par:
                        a, b = par.split("=", 1)
                        try:
                            meta[a] = float(b)
                        except ValueError:
                            meta[a] = b
                continue
            if linea.startswith("dt_us"):
                continue
            filas.append([float(x) for x in linea.split(",")])
    if not filas:
        sys.exit(f"{ruta}: sin datos")
    a = np.array(filas)
    d = dict(dt=a[:, 0] / 1e6, exc=a[:, 1], u=a[:, 2], th=a[:, 3], w=a[:, 4],
             thr=a[:, 5], sat=a[:, 6].astype(bool))
    d["fs"] = float(meta.get("fs", 500))
    d["Ts"] = 1.0 / d["fs"]
    d["t"] = np.cumsum(d["dt"]) - d["dt"][0]
    d["meta"] = meta
    d["sigma"] = 1.0 if meta.get("eje", "r") == "r" else -1.0
    return d


# ----------------------------------------------------------------------------- filtros
def butter2(fc, fs):
    """Butterworth de 2.o orden pasabajos (bilineal)."""
    k = np.tan(np.pi * fc / fs)
    q = np.sqrt(2.0)
    n = 1 / (1 + q * k + k * k)
    b = np.array([k * k * n, 2 * k * k * n, k * k * n])
    a = np.array([1.0, 2 * (k * k - 1) * n, (1 - q * k + k * k) * n])
    return b, a


def lfilter(b, a, x):
    y = np.zeros_like(x)
    x1 = x2 = y1 = y2 = 0.0
    x1 = x2 = x[0]
    y1 = y2 = x[0] * b.sum() / a.sum()
    for i, xi in enumerate(x):
        yi = b[0] * xi + b[1] * x1 + b[2] * x2 - a[1] * y1 - a[2] * y2
        x2, x1, y2, y1 = x1, xi, y1, yi
        y[i] = yi
    return y


def filtfilt(b, a, x):
    return lfilter(b, a, lfilter(b, a, x)[::-1])[::-1]


def retraso(x, d):
    if d == 0:
        return x.copy()
    return np.concatenate([np.full(d, x[0]), x[:-d]])


def primer_orden(u, tau, Ts):
    al = np.exp(-Ts / tau)
    y = np.empty_like(u)
    acc = u[0]
    for i, ui in enumerate(u):
        acc = al * acc + (1 - al) * ui
        y[i] = acc
    return y


# ----------------------------------------------------------------------------- identificacion
def identificar(D, fc=12.0, borde_s=0.5):
    Ts, fs = D["Ts"], D["fs"]
    b, a = butter2(fc, fs)
    thf = filtfilt(b, a, D["th"])
    wf = filtfilt(b, a, D["w"])
    alfa = np.gradient(wf, Ts)
    m = np.ones(len(thf), bool)
    nb = int(borde_s * fs)
    m[:nb] = False
    m[-nb:] = False
    m &= ~D["sat"]
    taus = np.arange(0.010, 0.2001, 0.005)
    mejor = None
    for tau in taus:
        uf0 = filtfilt(b, a, primer_orden(D["u"], tau, Ts))
        for d in range(0, 21):
            ufd = retraso(uf0, d)
            X = np.column_stack([ufd, wf, thf, np.ones_like(thf)])[m]
            y = alfa[m]
            th_, *_ = np.linalg.lstsq(X, y, rcond=None)
            res = y - X @ th_
            sse = float(res @ res)
            if mejor is None or sse < mejor[0]:
                mejor = (sse, tau, d, th_)
    sse, tau, d, (K, mb, mk, c) = mejor
    y = alfa[m]
    r2 = 1 - sse / float(((y - y.mean()) ** 2).sum())
    P = dict(K=float(K), b=float(-mb), k=float(-mk), c=float(c), tau=float(tau), d=int(d),
             Ts=Ts, r2=float(r2), fc=fc)
    P["_debug"] = dict(alfa=alfa, mask=m)
    return P


def sim_abierto(P, ud, th0, w0, uf0):
    """Modelo continuo (4 subpasos por muestra) con el mando YA retrasado ud."""
    h = P["Ts"] / 4
    al = np.exp(-h / P["tau"])
    th, w, uf = th0, w0, uf0
    out = np.empty(len(ud))
    for i, ui in enumerate(ud):
        for _ in range(4):
            uf = al * uf + (1 - al) * ui
            w += h * (P["K"] * uf - P["b"] * w - P["k"] * th + P["c"])
            th += h * w
        out[i] = th
    return out


def fit_pct(med, pred):
    return 100 * (1 - np.linalg.norm(med - pred) / max(np.linalg.norm(med - med.mean()), 1e-9))


def validar_ventanas(D, P, ventana_s=1.0):
    """Simula el modelo solo (sin controlador) en ventanas de 1 s, arrancando cada una
    desde el estado medido, y compara el angulo."""
    n = int(ventana_s * D["fs"])
    ud = retraso(D["u"], P["d"])
    uf = primer_orden(ud, P["tau"], P["Ts"])
    fits, pares = [], []
    for i0 in range(int(0.5 * D["fs"]), len(D["th"]) - n, n):
        pred = sim_abierto(P, ud[i0:i0 + n], D["th"][i0], D["w"][i0], uf[i0 - 1])
        fits.append(fit_pct(D["th"][i0:i0 + n], pred))
        pares.append((i0, pred))
    return np.array(fits), pares


# ----------------------------------------------------------------------------- lazo cerrado
def sim_cerrado(P, sigma, kp, kd, ki, exc, th0, w0, n):
    """Simula el firmware (PID con D por giro y media movil) + modelo, con la excitacion
    registrada. Sin banda muerta ni saturacion."""
    Ts = P["Ts"]
    h = Ts / 4
    al = np.exp(-h / P["tau"])
    th, w, uf, integ = th0, w0, 0.0, 0.0
    buf_w = [w0] * N_MA
    buf_u = [0.0] * (P["d"] + 1)
    TH = np.empty(n)
    U = np.empty(n)
    for i in range(n):
        buf_w = buf_w[1:] + [w]
        e = sigma * th
        de = sigma * np.mean(buf_w)
        u = kp * e + kd * de + integ + exc[i]
        integ += ki * e * Ts
        buf_u = buf_u[1:] + [u]
        ud = buf_u[0]
        for _ in range(4):
            uf = al * uf + (1 - al) * ud
            w += h * (P["K"] * uf - P["b"] * w - P["k"] * th + P["c"])
            th += h * w
        TH[i] = th
        U[i] = u
    return TH, U


# ----------------------------------------------------------------------------- diseno
def expm(A):
    nrm = np.linalg.norm(A, 1)
    s = max(0, int(np.ceil(np.log2(nrm))) + 1) if nrm > 0.5 else 0
    As = A / 2 ** s
    E = np.eye(len(A))
    T = np.eye(len(A))
    for j in range(1, 14):
        T = T @ As / j
        E = E + T
    for _ in range(s):
        E = E @ E
    return E


def planta_discreta(P):
    """x = [th, w, uf], entrada v (mando ya retrasado), ZOH a Ts."""
    Ac = np.array([[0, 1, 0], [-P["k"], -P["b"], P["K"]], [0, 0, -1 / P["tau"]]])
    Bc = np.array([[0], [0], [1 / P["tau"]]])
    M = np.zeros((4, 4))
    M[:3, :3] = Ac
    M[:3, 3:] = Bc
    E = expm(M * P["Ts"])
    return E[:3, :3], E[:3, 3]


def lazo_cerrado(P, sigma, kp, kd, ki=0.0, g=1.0, d_extra=0):
    """Matriz del lazo cerrado discreto. Estados: [th, w, uf, w_{k-1..k-4}, u_{k-1..k-d}, I].
    El mando u_k = g*sigma*(kp*th + kd*media(w) ) + I se aplica d muestras despues."""
    Ad, Bd = planta_discreta(P)
    d = P["d"] + d_extra
    nw = N_MA - 1
    con_i = ki != 0.0
    n = 3 + nw + d + (1 if con_i else 0)
    iI = n - 1 if con_i else None
    # fila de control: u_k = F x_k
    F = np.zeros(n)
    F[0] = g * sigma * kp
    F[1] = g * sigma * kd / N_MA
    F[3:3 + nw] = g * sigma * kd / N_MA
    if con_i:
        F[iI] = 1.0
    A = np.zeros((n, n))
    v = F if d == 0 else np.eye(n)[3 + nw + d - 1]   # mando que entra a la planta
    A[:3, :] = np.outer(Bd, v)
    A[:3, :3] += Ad
    # historial del giro
    if nw:
        A[3, 1] = 1.0
        for j in range(1, nw):
            A[3 + j, 3 + j - 1] = 1.0
    # buffer de mando
    if d:
        A[3 + nw, :] = F
        for j in range(1, d):
            A[3 + nw + j, 3 + nw + j - 1] = 1.0
    # integrador: I_{k+1} = I_k + g*ki*sigma*th*Ts
    if con_i:
        A[iI, iI] = 1.0
        A[iI, 0] += g * ki * sigma * P["Ts"]
    return A, F


def radio(A):
    return float(np.max(np.abs(np.linalg.eigvals(A))))


def margenes(P, sigma, kp, kd, ki=0.0):
    if radio(lazo_cerrado(P, sigma, kp, kd, ki)[0]) >= 1:
        return dict(estable=False)
    # margen de ganancia hacia arriba (factor) y hacia abajo
    lo, hi = 1.0, 20.0
    if radio(lazo_cerrado(P, sigma, kp, kd, ki, g=hi)[0]) < 1:
        gm = hi
    else:
        for _ in range(30):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if radio(lazo_cerrado(P, sigma, kp, kd, ki, g=mid)[0]) < 1 else (lo, mid)
        gm = lo
    # margen de retardo
    dm = 0
    while dm < 100 and radio(lazo_cerrado(P, sigma, kp, kd, ki, d_extra=dm + 1)[0]) < 1:
        dm += 1
    # margen de fase por respuesta en frecuencia del lazo abierto
    w = np.logspace(-1, np.log10(np.pi / P["Ts"] * 0.99), 4000)
    z = np.exp(1j * w * P["Ts"])
    s = (z - 1) / P["Ts"]   # aproximacion para la planta continua
    Pz = P["K"] / ((P["tau"] * s + 1) * (s ** 2 + P["b"] * s + P["k"])) * z ** (-P["d"])
    ma = sum(z ** (-j) for j in range(N_MA)) / N_MA
    C = kp + kd * s * ma
    L = -sigma * C * Pz
    cruces = np.where(np.diff(np.sign(np.abs(L) - 1)))[0]
    pm = None
    wc = None
    if len(cruces):
        i = cruces[-1]
        wc = float(w[i])
        pm = float(((np.angle(L[i], deg=True) + 180) + 180) % 360 - 180)
    return dict(estable=True, gm=gm, gm_db=20 * np.log10(gm), dm_s=dm * P["Ts"],
                pm=pm, fc_hz=None if wc is None else wc / 2 / np.pi)


def disenar(P, sigma, th0=10.0, t_sim=3.0, gm_min=2.0, dm_min=0.015, pm_min=40.0, ki=0.0):
    kps = np.geomspace(0.0003, KP_MAX, 50)
    kds = np.concatenate([[0], np.geomspace(2e-5, KD_MAX, 45)])
    cand = [(kp, kd) for kp in kps for kd in kds]
    As, Fs = [], []
    for kp, kd in cand:
        A, F = lazo_cerrado(P, sigma, kp, kd, ki)
        As.append(A)
        Fs.append(F)
    As = np.array(As)
    Fs = np.array(Fs)
    rad = np.array([radio(A) for A in As])
    n_st = As.shape[1]
    x = np.zeros((len(cand), n_st))
    x[:, 0] = th0
    nk = int(t_sim / P["Ts"])
    itae = np.zeros(len(cand))
    upk = np.zeros(len(cand))
    for k in range(nk):
        itae += (k * P["Ts"]) * np.abs(x[:, 0]) * P["Ts"]
        upk = np.maximum(upk, np.abs(np.einsum("ij,ij->i", Fs, x)))
        x = np.einsum("nij,nj->ni", As, x)
    orden = np.argsort(np.where(rad < 0.999, itae, np.inf))
    for i in orden:
        if rad[i] >= 0.999:
            break
        kp, kd = cand[i]
        mg = margenes(P, sigma, kp, kd, ki)
        if (mg["estable"] and mg["gm"] >= gm_min and mg["dm_s"] >= dm_min and upk[i] < 0.35
                and mg["pm"] is not None and mg["pm"] >= pm_min):
            return dict(kp=float(kp), kd=float(kd), ki=ki, itae=float(itae[i]),
                        u_pico=float(upk[i]), **mg)
    return None


# ----------------------------------------------------------------------------- informe
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--validar", help="otro CSV para validar el modelo en lazo cerrado")
    ap.add_argument("--gm", type=float, default=2.0, help="margen de ganancia minimo (factor)")
    ap.add_argument("--pm", type=float, default=40.0, help="margen de fase minimo [grados]")
    ap.add_argument("--dm", type=float, default=0.015, help="margen de retardo minimo [s]")
    args = ap.parse_args()

    D = leer_csv(args.csv)
    sal = Path(args.csv).with_suffix("")
    meta = D["meta"]
    sigma = D["sigma"]
    print(f"Ensayo: eje {meta.get('eje')}  {len(D['th'])} muestras  {D['t'][-1]:.1f} s  "
          f"Kp={meta.get('kp')} Kd={meta.get('kd')} Ki={meta.get('ki')} thr={meta.get('thr')}")
    print(f"  saturado {100 * D['sat'].mean():.1f} % del tiempo;  |angulo| max {np.abs(D['th']).max():.1f} grados;"
          f"  fin: {meta.get('fin')}")
    if D["sat"].mean() > 0.05:
        print("  AVISO: mas de 5 % saturado: bajar la amplitud o subir el acelerador.")

    P = identificar(D)
    print("\nModelo identificado:")
    print(f"  K   = {P['K']:9.1f} grados/s^2 por unidad de mando")
    print(f"  b   = {P['b']:9.3f} 1/s      k = {P['k']:8.3f} 1/s^2      c = {P['c']:8.2f} grados/s^2")
    print(f"  tau = {P['tau'] * 1000:6.0f} ms     retardo = {P['d']} muestras ({P['d'] * P['Ts'] * 1000:.0f} ms)")
    print(f"  R^2 de la aceleracion = {P['r2']:.3f}")
    if P["K"] * sigma > 0:
        print("  AVISO: el signo de K indica que la P del firmware EMPUJA en vez de corregir en este eje.")

    fits, pares = validar_ventanas(D, P)
    print(f"  Ajuste del angulo en ventanas de 1 s (modelo solo): mediana {np.median(fits):.0f} %")

    kp0, kd0, ki0 = (float(meta.get(x, 0)) for x in ("kp", "kd", "ki"))
    thc, _ = sim_cerrado(P, sigma, kp0, kd0, ki0, D["exc"], D["th"][0], D["w"][0], len(D["th"]))
    fit_lc = fit_pct(D["th"], thc)
    print(f"  Lazo cerrado completo con las ganancias del ensayo: ajuste {fit_lc:.0f} %")
    mg0 = margenes(P, sigma, kp0, kd0, ki0)
    if mg0["estable"]:
        print(f"  Ganancias del ensayo: MG x{mg0['gm']:.2f} ({mg0['gm_db']:.1f} dB), margen de retardo "
              f"{mg0['dm_s'] * 1000:.0f} ms, MF {mg0['pm'] if mg0['pm'] is None else round(mg0['pm'])} grados")
    else:
        print("  Ganancias del ensayo: INESTABLES segun el modelo")

    Dz = disenar(P, sigma, gm_min=args.gm, dm_min=args.dm, pm_min=args.pm)
    print("\nPD recomendado (respuesta a soltar desde 10 grados, menor ITAE con margenes):")
    if Dz is None:
        print("  Ninguna combinacion cumple los margenes. Revisar el ensayo.")
    else:
        print(f"  Kp = {Dz['kp']:.5f}   Kd = {Dz['kd']:.5f}   (Ki: empezar en 0 y subir hasta quitar el offset)")
        print(f"  MG x{Dz['gm']:.2f} ({Dz['gm_db']:.1f} dB), margen de retardo {Dz['dm_s'] * 1000:.0f} ms, "
              f"MF {Dz['pm'] if Dz['pm'] is None else round(Dz['pm'])} grados, "
              f"cruce {Dz['fc_hz'] if Dz['fc_hz'] is None else round(Dz['fc_hz'], 2)} Hz, mando pico {Dz['u_pico']:.3f}")
        if Dz["kp"] <= 0.00031 or Dz["kp"] >= KP_MAX * 0.99:
            print("  AVISO: quedo en el borde de la busqueda; el modelo o los margenes pedidos no calzan.")
        if abs(P["c"]) > 0 and Dz["kp"] > 0:
            off = P["c"] / (-sigma * P["K"] * Dz["kp"] + P["k"]) if (-sigma * P["K"] * Dz["kp"] + P["k"]) else float("nan")
            print(f"  Offset esperado por desbalance sin Ki: {off:.2f} grados")

    # ---------------- graficas
    fig, ax = plt.subplots(4, 1, figsize=(11, 12))
    t = D["t"]
    ax[0].plot(t, D["u"], lw=0.7, label="u (mezclador)")
    ax[0].plot(t, D["exc"], lw=0.7, label="excitacion")
    ax[0].set_ylabel("mando")
    ax[0].legend(loc="upper right")
    ax[0].set_title(f"Ensayo eje {meta.get('eje')}")
    m = P["_debug"]["mask"]
    uf = primer_orden(retraso(D["u"], P["d"]), P["tau"], P["Ts"])
    b, a = butter2(P["fc"], D["fs"])
    pred = P["K"] * filtfilt(b, a, uf) - P["b"] * filtfilt(b, a, D["w"]) - P["k"] * filtfilt(b, a, D["th"]) + P["c"]
    ax[1].plot(t, P["_debug"]["alfa"], lw=0.7, label="medida (derivada del giro)")
    ax[1].plot(t, pred, lw=0.7, label=f"modelo  R2={P['r2']:.2f}")
    ax[1].set_ylabel("acel. angular [grados/s2]")
    ax[1].legend(loc="upper right")
    ax[2].plot(t, D["th"], "k", lw=0.8, label="medido")
    for i0, pr in pares:
        ax[2].plot(t[i0:i0 + len(pr)], pr, "r", lw=0.8)
    ax[2].plot([], [], "r", label=f"modelo en ventanas de 1 s (mediana {np.median(fits):.0f} %)")
    ax[2].set_ylabel("angulo [grados]")
    ax[2].legend(loc="upper right")
    ax[3].plot(t, D["th"], "k", lw=0.8, label="medido")
    ax[3].plot(t, thc, "r", lw=0.8, label=f"lazo cerrado simulado ({fit_lc:.0f} %)")
    ax[3].set_ylabel("angulo [grados]")
    ax[3].set_xlabel("t [s]")
    ax[3].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(f"{sal}_identificacion.png", dpi=120)

    if Dz is not None:
        n = int(3 / P["Ts"])
        tt = np.arange(n) * P["Ts"]
        fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        for kp, kd, ki, et in [(kp0, kd0, ki0, "actuales"), (Dz["kp"], Dz["kd"], 0.0, "recomendadas")]:
            th, u = sim_cerrado(P, sigma, kp, kd, ki, np.zeros(n), 10.0, 0.0, n)
            ax[0].plot(tt, th, label=f"{et}: Kp={kp:.4f} Kd={kd:.5f}")
            ax[1].plot(tt, u)
        ax[0].set_ylabel("angulo [grados]")
        ax[0].set_title("Prediccion: soltar el dron desde 10 grados")
        ax[0].legend()
        ax[0].grid(alpha=0.3)
        ax[1].set_ylabel("mando u")
        ax[1].set_xlabel("t [s]")
        ax[1].grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(f"{sal}_diseno.png", dpi=120)

    if args.validar:
        V = leer_csv(args.validar)
        vm = V["meta"]
        kv = [float(vm.get(x, 0)) for x in ("kp", "kd", "ki")]
        thv, _ = sim_cerrado(P, V["sigma"], *kv, V["exc"], V["th"][0], V["w"][0], len(V["th"]))
        fv = fit_pct(V["th"], thv)
        print(f"\nValidacion con {args.validar} (Kp={kv[0]} Kd={kv[1]} Ki={kv[2]}): ajuste {fv:.0f} %")
        fig, ax = plt.subplots(figsize=(11, 4))
        ax.plot(V["t"], V["th"], "k", lw=0.8, label="medido")
        ax.plot(V["t"], thv, "r", lw=0.8, label=f"modelo ({fv:.0f} %)")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("angulo [grados]")
        ax.legend()
        fig.tight_layout()
        fig.savefig(f"{Path(args.validar).with_suffix('')}_validacion.png", dpi=120)

    res = {k: v for k, v in P.items() if not k.startswith("_")}
    res.update(fit_ventanas_mediana=float(np.median(fits)), fit_lazo_cerrado=float(fit_lc),
               margenes_actuales=mg0, recomendado=Dz)
    Path(f"{sal}_resultado.json").write_text(json.dumps(res, indent=2, default=float), encoding="utf-8")
    print(f"\nGraficas y resultados: {sal}_identificacion.png, {sal}_diseno.png, {sal}_resultado.json")


if __name__ == "__main__":
    main()
