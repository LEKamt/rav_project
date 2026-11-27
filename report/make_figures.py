"""
Genera las figuras del informe a partir de los datos reales del proyecto.

  python3 make_figures.py

Entradas
  ../Identificación de parámetros Drone.xlsx   hojas INE (péndulo bifilar) y Kf (banco de empuje)
  ../../Data_recortada/*.csv                   cajas negras recortadas (V10, V10b, V11)

Salidas (en figures/)
  f_arquitectura.png  f_ident.png  f_vuelo_B.png  f_hold_A.png  f_estimador.png

Además imprime por consola las métricas que se citan en las tablas del informe,
para que cada número del texto se pueda rastrear a esta salida.
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import openpyxl

AQUI = os.path.dirname(os.path.abspath(__file__))
XLSX = os.path.join(AQUI, "..", "Identificación de parámetros Drone.xlsx")
DATA = os.path.join(AQUI, "..", "..", "Data_recortada")
OUT = os.path.join(AQUI, "figures")

plt.rcParams.update({
    "font.family": "serif", "mathtext.fontset": "stix",
    "font.size": 7.5, "axes.labelsize": 7.5, "axes.titlesize": 7.5,
    "legend.fontsize": 6.5, "xtick.labelsize": 7, "ytick.labelsize": 7,
    "axes.grid": True, "grid.alpha": 0.25, "grid.linewidth": 0.5,
    "axes.linewidth": 0.6, "lines.linewidth": 0.9, "figure.dpi": 100,
    "savefig.dpi": 300, "savefig.bbox": "tight", "savefig.pad_inches": 0.03,
})
C = {"azul": "#1f5a8a", "naranja": "#d1691a", "verde": "#2e8b57", "rojo": "#b8322a",
     "gris": "#6b6b6b", "morado": "#6a4c93", "negro": "#222222"}
W1, W2 = 3.45, 7.15  # ancho de columna y de página IEEE (pulgadas)
G = 9.81


def cargar(tag):
    return np.genfromtxt(os.path.join(DATA, tag + ".csv"), delimiter=",", names=True)


def con_huecos(t, y, gap=0.1):
    """Inserta NaN donde el recorte dejó un hueco, para no unir tramos distintos."""
    t = np.asarray(t, float).copy()
    y = np.asarray(y, float).copy()
    idx = np.where(np.diff(t) > gap)[0] + 1
    return np.insert(t, idx, np.nan), np.insert(y, idx, np.nan)


def etiqueta(ax, s, x=0.012, y=0.93, ha="left"):
    ax.text(x, y, s, transform=ax.transAxes, fontsize=7.5, fontweight="bold", va="top", ha=ha)


# ----------------------------------------------------------------------------
# 1. Identificación: banco de empuje y péndulo bifilar
# ----------------------------------------------------------------------------
def fig_ident():
    wb = openpyxl.load_workbook(XLSX, data_only=True)
    ws = wb["Kf"]
    pwm = np.array([ws.cell(r, 2).value for r in range(3, 19)], float)
    gr = np.array([ws.cell(r, 3).value for r in range(3, 19)], float)
    F = gr / 1000.0 * G
    usado = np.array([ws.cell(r, 5).value for r in range(22, 38)], bool)   # columna "Filtrar"
    a, b, c = (ws["B41"].value, ws["C41"].value, ws["D41"].value)
    pwm0 = -b / (2 * a)
    cp = c - b * b / (4 * a)
    kf_hoja = ws["C60"].value
    ic_hoja = ws["D60"].value

    x = np.linspace(1000, 2000, 400)
    cuad = a * x ** 2 + b * x + c
    lin = kf_hoja * (x - pwm0) ** 2 + ic_hoja + cp

    fig, ax = plt.subplots(1, 2, figsize=(W2, 2.35), gridspec_kw={"width_ratios": [1.25, 1]})
    A = ax[0]
    A.plot(x, 4 * cuad, color=C["azul"], label=r"ajuste cuadr. (13 pts): $a=%.2f\times10^{-6}$" % (a * 1e6))
    A.plot(x, 4 * lin, color=C["naranja"], ls="--",
           label=r"hoja $K_f$ (1000–1500 µs): $%.2f\times10^{-6}$ N/µs$^2$" % (kf_hoja * 1e6))
    A.scatter(pwm[usado], 4 * F[usado], s=11, color=C["negro"], zorder=3, label="medidas usadas en el ajuste (×4)")
    A.scatter(pwm[~usado], 4 * F[~usado], s=14, facecolors="none", edgecolors=C["rojo"], zorder=3,
              label="medidas descartadas (×4)")
    A.axhline(1.0022 * G, color=C["gris"], ls=":", lw=0.9)
    A.text(1010, 1.0022 * G + 0.25, r"peso $mg=9.83$ N", color=C["gris"], fontsize=6.5)
    A.axvspan(1040, 1800, color=C["verde"], alpha=0.08, lw=0)
    A.text(1045, 12.4, "recorrido de vuelo\n1040–1800 µs", color=C["verde"], fontsize=6.3, va="top")
    A.set_xlabel("ancho de pulso PWM [µs]")
    A.set_ylabel("empuje total de 4 motores [N]")
    A.set_xlim(1000, 2000)
    A.set_ylim(-0.3, 14.5)
    A.legend(loc="upper left", bbox_to_anchor=(0.0, 0.70), frameon=False, handlelength=1.6)
    etiqueta(A, "(a)", y=0.99)

    # péndulo bifilar: períodos medidos por eje
    ws = wb["INE"]
    ejes = [("$x$", 20, C["azul"]), ("$y$", 36, C["naranja"]), ("$z$", 4, C["verde"])]
    B = ax[1]
    for i, (nom, r0, col) in enumerate(ejes):
        T = np.array([ws.cell(r0 + k, 4).value for k in range(5)], float)
        B.scatter(np.full(5, i) + np.linspace(-0.12, 0.12, 5), T, s=12, color=col, zorder=3)
        B.errorbar(i + 0.28, T.mean(), yerr=T.std(ddof=1), fmt="s", ms=4, color=C["negro"], capsize=2, lw=0.8)
    B.set_xticks(range(3))
    B.set_xticklabels([e[0] for e in ejes])
    B.set_xlim(-0.5, 2.6)
    B.set_xlabel("eje del péndulo bifilar")
    B.set_ylabel("período de oscilación $T$ [s]")
    B.text(0.03, 0.62, "puntos: 5 ensayos de 5 oscilaciones\ncuadrado: media ± desv. est.", transform=B.transAxes,
           fontsize=6.3, color=C["gris"])
    etiqueta(B, "(b)", y=0.99)
    fig.tight_layout(w_pad=1.2)
    fig.savefig(os.path.join(OUT, "f_ident.png"))
    plt.close(fig)


# ----------------------------------------------------------------------------
# 2. Arquitectura de control
# ----------------------------------------------------------------------------
def caja(ax, x, y, w, h, txt, fc="#eef3f8", ec=C["azul"], fs=6.6, lw=0.8, bold=False):
    p = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.0,rounding_size=0.06", fc=fc, ec=ec, lw=lw)
    ax.add_patch(p)
    ax.text(x + w / 2, y + h / 2, txt, ha="center", va="center", fontsize=fs,
            fontweight="bold" if bold else "normal", linespacing=1.25)


def flecha(ax, p0, p1, txt=None, off=(0, 0.1), fs=6.2, color=C["negro"], ls="-", ha="center", rad=0.0):
    ar = FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=7, lw=0.8, color=color, ls=ls,
                         connectionstyle="arc3,rad=%g" % rad, shrinkA=0, shrinkB=0)
    ax.add_patch(ar)
    if txt:
        ax.text((p0[0] + p1[0]) / 2 + off[0], (p0[1] + p1[1]) / 2 + off[1], txt, fontsize=fs, ha=ha,
                va="bottom", color=color)


def fig_arquitectura():
    fig, ax = plt.subplots(figsize=(W2, 3.2))
    ax.set_xlim(0, 14.3)
    ax.set_ylim(0, 6.3)
    ax.axis("off")
    ax.grid(False)
    fs = 6.1
    V, N_, G_, A_ = "#eef6ee", "#eef3f8", "#f4f4f4", "#fbf1e6"

    # fila superior: altura
    caja(ax, 0.1, 4.55, 1.8, 1.05, "Página web\n(WiFi)\n$z_{ref},\\eta_d$", fc=G_, ec=C["gris"], fs=fs)
    caja(ax, 2.4, 4.55, 2.5, 1.05, "Lazo de altura\nec. (35)–(36), $g_{ef}$\n$u_1=(v_z-g)/r_{33}$", fc=V, ec=C["verde"], fs=fs)
    caja(ax, 5.4, 4.55, 2.3, 1.05, "Kalman vertical\n$[z,\\dot z,b]$", fc=V, ec=C["verde"], fs=fs)
    caja(ax, 8.2, 4.55, 2.7, 1.05, "HC-SR04 (mediana de 3)\nVL53L0X (< 25 cm)", fc=G_, ec=C["gris"], fs=fs)
    caja(ax, 11.4, 4.55, 2.8, 1.05, "Super-twisting (opcional)\n$w\\to$ perturbación", fc=A_, ec=C["naranja"], fs=fs)
    flecha(ax, (1.9, 5.07), (2.4, 5.07))
    flecha(ax, (5.4, 5.07), (4.9, 5.07))
    flecha(ax, (8.2, 5.07), (7.7, 5.07))
    ax.plot([12.8, 12.8, 3.65], [5.6, 5.95, 5.95], color=C["naranja"], lw=0.8, ls="--")
    flecha(ax, (3.65, 5.95), (3.65, 5.6), None, color=C["naranja"], ls="--")
    ax.text(8.2, 6.0, "$+w$ en $v_z$", fontsize=6.0, color=C["naranja"], ha="center", va="bottom")

    # fila media: actitud
    caja(ax, 0.1, 2.45, 2.1, 1.15, "Lazo externo\n$\\dot\\eta_d=k_{p,\\eta}(\\eta_d-\\eta)$\nec. (40), (44)", fc=N_, fs=fs)
    caja(ax, 2.6, 2.45, 2.4, 1.15, "Inversión de ec. (26)\n$\\dot\\eta_d\\to(p_d,q_d,r_d)$", fc=N_, fs=fs)
    caja(ax, 5.5, 2.45, 2.5, 1.15, "Lazo interno, ec. (42)\n$v=k_p(\\omega_d-\\omega)$\n$+\\int$ ó super-twisting", fc=N_, fs=fs)
    caja(ax, 8.5, 2.45, 2.4, 1.15, "Linealización, ec. (41)\n$\\tau=I\\,v+\\omega\\times I\\omega$", fc=N_, fs=fs)
    caja(ax, 11.4, 2.45, 2.8, 1.15, "Identificación RLS\n$g=\\dot\\omega/v$,  $k_p=1/(\\tau_{des}\\,g)$", fc=A_, ec=C["naranja"], fs=fs)
    flecha(ax, (1.0, 4.55), (1.0, 3.6))
    ax.text(1.08, 4.0, "$\\eta_d$", fontsize=6.0)
    flecha(ax, (2.2, 3.02), (2.6, 3.02))
    flecha(ax, (5.0, 3.02), (5.5, 3.02))
    flecha(ax, (8.0, 3.02), (8.5, 3.02))
    flecha(ax, (10.9, 3.02), (11.4, 3.02), color=C["naranja"], ls="--")

    # fila inferior
    caja(ax, 0.1, 0.3, 2.6, 1.2, "Mahony (cuaternión)\n$\\phi,\\theta,\\psi$ ; $p,q,r$", fc=G_, ec=C["gris"], fs=fs)
    caja(ax, 3.2, 0.3, 2.2, 1.2, "IMU MPU-6500\n250 Hz", fc=G_, ec=C["gris"], fs=fs)
    caja(ax, 6.0, 0.3, 3.6, 1.2, "Mezclador, ec. (46)\nroll/pitch antes que yaw\nrecorte al politopo", fc=N_, fs=fs)
    caja(ax, 10.1, 0.3, 1.9, 1.2, "PWM 50 Hz\n4 ESC 30 A", fc=G_, ec=C["gris"], fs=fs)
    caja(ax, 12.5, 0.3, 1.7, 1.2, "F450\nA2212", fc=G_, ec=C["gris"], fs=fs)
    flecha(ax, (3.2, 0.9), (2.7, 0.9))
    flecha(ax, (1.3, 1.5), (1.3, 2.45))
    ax.text(1.38, 1.85, "$\\eta,\\omega$", fontsize=6.0)
    flecha(ax, (9.6, 0.9), (10.1, 0.9))
    flecha(ax, (12.0, 0.9), (12.5, 0.9))

    # empuje y torques hacia el mezclador
    ax.plot([3.65, 3.65, 8.25], [4.55, 4.1, 4.1], color=C["negro"], lw=0.8)
    flecha(ax, (8.25, 4.1), (8.25, 1.5))
    ax.text(5.9, 4.15, "$f_t=-m\\,u_1$", fontsize=6.0, ha="center", va="bottom")
    flecha(ax, (9.2, 2.45), (9.2, 1.5))
    ax.text(9.28, 1.95, "$\\tau$", fontsize=6.0)
    # omega de la IMU al lazo interno
    ax.plot([4.3, 4.3, 6.2], [1.5, 2.0, 2.0], color=C["negro"], lw=0.8)
    flecha(ax, (6.2, 2.0), (6.2, 2.45))
    ax.text(4.38, 1.68, "$\\omega$", fontsize=6.0)
    # a_z de la IMU al Kalman
    ax.plot([5.25, 5.25, 6.5], [1.5, 4.3, 4.3], color=C["verde"], lw=0.8)
    flecha(ax, (6.5, 4.3), (6.5, 4.55), color=C["verde"])
    ax.text(5.32, 3.78, "$a_z$", fontsize=6.0, color=C["verde"])
    # ganancia adaptada
    ax.plot([12.8, 12.8, 7.0], [2.45, 2.2, 2.2], color=C["naranja"], lw=0.8, ls="--")
    flecha(ax, (7.0, 2.2), (7.0, 2.45), color=C["naranja"], ls="--")
    ax.text(10.6, 2.27, "$k_p$ adaptada", fontsize=6.0, color=C["naranja"], ha="center", va="bottom")
    fig.savefig(os.path.join(OUT, "f_arquitectura.png"))
    plt.close(fig)


# ----------------------------------------------------------------------------
# 3. Vuelo B (V11, 19:25:45): subida, vuelo y toma de contacto
# ----------------------------------------------------------------------------
def fig_vuelo_B(d):
    t = d["t_vuelo_s"]
    m = t <= 22.0
    fig, ax = plt.subplots(4, 1, figsize=(W1, 5.6), sharex=True,
                           gridspec_kw={"height_ratios": [1.15, 1, 1, 1]})
    # (a) altura
    A = ax[0]
    tt, y = con_huecos(t[m], d["alt_m"][m])
    A.plot(tt, y, color=C["azul"], label="estimada (Kalman)")
    ok = m & np.isfinite(d["us_crudo_m"]) & (d["us_crudo_m"] > 0)
    A.plot(t[ok], d["us_crudo_m"][ok], ".", ms=1.6, color=C["naranja"], alpha=0.7, label="HC-SR04 crudo")
    okt = m & np.isfinite(d["tof_m"])
    A.plot(t[okt], d["tof_m"][okt], ".", ms=2.0, color=C["verde"], label="VL53L0X")
    A.axvspan(19.4, 22.0, color=C["rojo"], alpha=0.10, lw=0)
    A.annotate("estimador a la deriva\n(sin medida válida)", xy=(21.2, 0.95), xytext=(12.3, 1.45), fontsize=6.2,
               color=C["rojo"], arrowprops=dict(arrowstyle="-|>", color=C["rojo"], lw=0.7))
    A.set_ylabel("altura [m]")
    A.set_ylim(-0.1, 2.0)
    A.legend(loc="upper left", ncol=1, frameon=False, handlelength=1.4, borderaxespad=0.2)
    etiqueta(A, "(a)", x=0.995, ha="right")
    # (b) actitud
    B = ax[1]
    for k, col, lab in [("roll_deg", C["azul"], r"$\phi$"), ("pitch_deg", C["naranja"], r"$\theta$"),
                        ("yaw_deg", C["verde"], r"$\psi$")]:
        tt, y = con_huecos(t[m], d[k][m])
        B.plot(tt, y, color=col, label=lab, lw=0.8)
    B.axhline(60, color=C["rojo"], ls=":", lw=0.8)
    B.text(0.4, 60.8, "límite de desarme 60°", fontsize=6.0, color=C["rojo"])
    B.set_ylabel("ángulos [°]")
    B.set_ylim(-20, 70)
    B.legend(loc="upper right", ncol=3, frameon=False, handlelength=1.2, borderaxespad=0.2, bbox_to_anchor=(1.0, 0.82))
    etiqueta(B, "(b)", x=0.995, y=0.99, ha="right")
    # (c) velocidades angulares
    Cc = ax[2]
    for k, col, lab in [("p_dps", C["azul"], "$p$"), ("q_dps", C["naranja"], "$q$")]:
        tt, y = con_huecos(t[m], d[k][m])
        Cc.plot(tt, y, color=col, label=lab, lw=0.4, alpha=0.9)
    Cc.set_ylabel("$p,\\,q$ [°/s]")
    Cc.legend(loc="lower right", ncol=2, frameon=False, handlelength=1.2, borderaxespad=0.2)
    etiqueta(Cc, "(c)", x=0.995, y=0.99, ha="right")
    # (d) mandos de motor y empuje
    D = ax[3]
    for i, col in zip(range(1, 5), [C["azul"], C["naranja"], C["verde"], C["morado"]]):
        tt, y = con_huecos(t[m], d["m%d_pc" % i][m])
        D.plot(tt, y, color=col, lw=0.45, label="M%d" % i)
    D.set_ylabel("mando de motor [%]")
    D.set_ylim(0, 112)
    D.set_xlabel("tiempo de vuelo [s]")
    D.legend(loc="lower right", ncol=4, frameon=False, handlelength=1.1, borderaxespad=0.2, columnspacing=0.8)
    etiqueta(D, "(d)", x=0.995, y=0.99, ha="right")
    fig.align_ylabels(ax)
    fig.tight_layout(h_pad=0.4)
    fig.savefig(os.path.join(OUT, "f_vuelo_B.png"))
    plt.close(fig)


# ----------------------------------------------------------------------------
# 4. Vuelo A (V11, 19:09:36): modo MANTENER ALTURA
# ----------------------------------------------------------------------------
def fig_hold_A(d):
    t = d["t_vuelo_s"]
    m = t <= 4.2
    fig, ax = plt.subplots(3, 1, figsize=(W1, 4.1), sharex=True, gridspec_kw={"height_ratios": [1.25, 1, 1]})
    A = ax[0]
    A.plot(t[m], d["alt_m"][m], color=C["azul"], label="estimada")
    A.plot(t[m], d["alt_ref_m"][m], color=C["negro"], ls="--", label="referencia interna")
    ok = m & np.isfinite(d["us_crudo_m"]) & (d["us_crudo_m"] > 0)
    A.plot(t[ok], d["us_crudo_m"][ok], ".", ms=2.2, color=C["naranja"], alpha=0.8, label="HC-SR04 crudo")
    hold = d["alt_modo"] == 1
    t0, t1 = t[hold][0], t[hold][-1]
    for a_ in ax:
        a_.axvspan(t0, t1, color=C["verde"], alpha=0.10, lw=0)
    A.text((t0 + t1) / 2, 1.74, "MANTENER ALTURA", ha="center", fontsize=6.3, color=C["verde"])
    A.set_ylabel("altura [m]")
    A.set_ylim(0, 1.95)
    A.legend(loc="upper left", frameon=False, handlelength=1.4, borderaxespad=0.2, fontsize=6.2)
    etiqueta(A, "(a)", x=0.995, y=0.99, ha="right")
    B = ax[1]
    B.plot(t[m], d["vz_ms"][m], color=C["azul"])
    B.axhline(0, color=C["gris"], lw=0.6)
    B.set_ylabel("$\\dot z$ estimada [m/s]")
    etiqueta(B, "(b)", x=0.995, y=0.99, ha="right")
    Cc = ax[2]
    Cc.plot(t[m], d["ft_N"][m], color=C["azul"], label="$f_t$ pedido")
    Cc.axhline(1.0022 * G, color=C["gris"], ls=":", label="$mg$")
    Cc.set_ylabel("empuje [N]")
    Cc.set_xlabel("tiempo de vuelo [s]")
    Cc.legend(loc="lower right", ncol=2, frameon=False, handlelength=1.4, borderaxespad=0.2)
    etiqueta(Cc, "(c)", x=0.995, y=0.99, ha="right")
    fig.align_ylabels(ax)
    fig.tight_layout(h_pad=0.4)
    fig.savefig(os.path.join(OUT, "f_hold_A.png"))
    plt.close(fig)


# ----------------------------------------------------------------------------
# 5. Estimador de altura: V10 frente a V11
# ----------------------------------------------------------------------------
def fig_estimador(d10, d11):
    fig, ax = plt.subplots(2, 1, figsize=(W1, 3.5))
    for A, d, tit, lab in [(ax[0], d10, "V10 (vuelo 16:50)", "(a)"), (ax[1], d11, "V11 (vuelo 19:25)", "(b)")]:
        t = d["t_vuelo_s"]
        m = t <= 70
        tt, y = con_huecos(t[m], d["alt_m"][m])
        A.plot(tt, y, color=C["azul"], label="altura estimada")
        ok = m & np.isfinite(d["us_m"]) & (d["us_m"] > 0.08)
        A.plot(t[ok], d["us_m"][ok], ".", ms=1.4, color=C["naranja"], alpha=0.6, label="HC-SR04")
        A.axhline(0, color=C["gris"], lw=0.6)
        A.set_ylabel("altura [m]")
        A.set_title(tit, fontsize=7, loc="left", pad=2)
        A.set_ylim(-4.2, 2.6)
        A.set_xlim(0, 70)
        etiqueta(A, lab, x=0.94, y=0.99)
    ax[0].legend(loc="lower right", frameon=False, handlelength=1.4, borderaxespad=0.2)
    ax[1].set_xlabel("tiempo de vuelo [s]")
    ax[1].set_xlim(0, 35)
    fig.align_ylabels(ax)
    fig.tight_layout(h_pad=0.5)
    fig.savefig(os.path.join(OUT, "f_estimador.png"))
    plt.close(fig)


# ----------------------------------------------------------------------------
# Métricas que se citan en el texto
# ----------------------------------------------------------------------------
def metricas():
    print("\n=== Métricas por vuelo (muestras con altura estimada > 0.15 m) ===")
    for tag in sorted(f[:-4] for f in os.listdir(DATA) if f.endswith(".csv")):
        d = cargar(tag)
        t = d["t_vuelo_s"]
        aire = d["alt_m"] > 0.15
        us = d["us_m"]
        okus = np.isfinite(us) & (us > 0.1)
        res = d["alt_m"][okus] - us[okus]
        neg = (d["alt_m"] < -0.05).mean() * 100
        print("\n%s  n=%d  t_fin=%.1f s" % (tag, len(t), t[-1]))
        print("  en el aire: %d muestras; alt máx %.2f m; muestras con altura<-5cm: %.1f %%" % (aire.sum(), np.nanmax(d["alt_m"]), neg))
        print("  residuo (alt_est - US): media %.2f  desv %.2f  |res|>0.5 m: %.1f %%" % (res.mean(), res.std(), 100 * (abs(res) > 0.5).mean()))
        if aire.sum() > 20:
            r, p = d["roll_deg"][aire], d["pitch_deg"][aire]
            print("  roll  media %.1f desv %.1f máx|.| %.1f ; pitch media %.1f desv %.1f máx|.| %.1f" % (
                r.mean(), r.std(), abs(r).max(), p.mean(), p.std(), abs(p).max()))
            print("  |roll|<10°: %.0f %%  |roll|<20°: %.0f %% ; |pitch|<10°: %.0f %%" % (
                100 * (abs(r) < 10).mean(), 100 * (abs(r) < 20).mean(), 100 * (abs(p) < 10).mean()))
            print("  sigma p,q,r = %.1f %.1f %.1f °/s" % (d["p_dps"][aire].std(), d["q_dps"][aire].std(), d["r_dps"][aire].std()))
            print("  mezclador saturado %.0f %% ; ft medio %.2f N ; thr medio %.2f" % (100 * d["saturado"][aire].mean(), d["ft_N"][aire].mean(), d["thr"][aire].mean()))
            mm = np.vstack([d["m%d_pc" % i][aire] for i in range(1, 5)])
            print("  mando medio %s ; fracción >=99 %%: %s" % (mm.mean(1).round(1), (100 * (mm >= 99).mean(1)).round(0)))
    d = cargar("V11_20261126_192545")
    t = d["t_vuelo_s"]
    i = np.argmax(d["roll_deg"])
    print("\nVuelo B: roll máx %.1f° en t=%.2f s (p=%.0f °/s)" % (d["roll_deg"][i], t[i], d["p_dps"][i]))
    w = (t > 3.0) & (t < 22)
    seg = d["roll_deg"][w]
    k = np.where(abs(seg) > 30)[0]
    print("  tiempo con |roll|>30°: %.2f s" % (0.02 * len(k)))
    print("  inicio del empuje (ft>2 N): %.2f s ; altura>0.3 m: %.2f s" % (t[d['ft_N'] > 2][0], t[d['alt_m'] > 0.3][0]))


def main():
    os.makedirs(OUT, exist_ok=True)
    fig_ident()
    fig_arquitectura()
    A = cargar("V11_20261126_190936")
    B = cargar("V11_20261126_192545")
    fig_vuelo_B(B)
    fig_hold_A(A)
    fig_estimador(cargar("V10_20261126_165034"), B)
    metricas()
    print("\nFiguras escritas en", OUT)


if __name__ == "__main__":
    main()
