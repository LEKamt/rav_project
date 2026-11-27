"""
Figuras añadidas al informe (27-nov):

  f_esquema.png          esquema de control (lazos, mezclador, estimación)
  f_hold_versiones.png   altura medida y referencia en MANTENER ALTURA, V_10 -> V_11

  python make_figures_hold.py

Usa las cajas negras recortadas de Presentacion_ravproject/Data_recortada y el
mismo estilo que make_figures.py. Imprime las métricas que cita el texto.
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

AQUI = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(AQUI, "..", "..", "Presentacion_ravproject", "Data_recortada")
OUT = os.path.join(AQUI, "figures")

plt.rcParams.update({
    "font.family": "serif", "mathtext.fontset": "stix",
    "font.size": 7.5, "axes.labelsize": 7.5, "axes.titlesize": 7.5,
    "legend.fontsize": 6.5, "xtick.labelsize": 7, "ytick.labelsize": 7,
    "axes.grid": True, "grid.alpha": 0.25, "grid.linewidth": 0.5,
    "axes.linewidth": 0.6, "lines.linewidth": 0.9, "figure.dpi": 100,
    "savefig.dpi": 300, "savefig.bbox": "tight", "savefig.pad_inches": 0.03,
})
C = {"azul": "#1f5a8a", "naranja": "#d1691a", "verde": "#2e8b57", "gris": "#6b6b6b", "negro": "#222222"}
W1, W2 = 3.45, 7.15


def cargar(tag):
    return np.genfromtxt(os.path.join(DATA, tag + ".csv"), delimiter=",", names=True)


# ----------------------------------------------------------------------------
# 1. Esquema de control
# ----------------------------------------------------------------------------
def caja(ax, x, y, w, h, txt, sub=None, fc="#f0f0f0", ec="#222222", ls="-", col="#222222", fs=7.2):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=0.12",
                                fc=fc, ec=ec, lw=0.9, ls=ls))
    if sub:
        ax.text(x + w / 2, y + h * 0.60, txt, ha="center", va="center", fontsize=fs, fontweight="bold", color=col)
        ax.text(x + w / 2, y + h * 0.28, sub, ha="center", va="center", fontsize=fs - 1.2, color=C["gris"])
    else:
        ax.text(x + w / 2, y + h / 2, txt, ha="center", va="center", fontsize=fs, fontweight="bold", color=col)


def flecha(ax, p0, p1, color="#222222", ls="-"):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=7, lw=0.8, color=color, ls=ls,
                                 shrinkA=0, shrinkB=0))


def linea(ax, xs, ys, color=C["gris"]):
    ax.plot(xs, ys, color=color, lw=0.8, solid_capstyle="butt")


def fig_esquema():
    fig, ax = plt.subplots(figsize=(W2, 2.75))
    ax.set_xlim(0, 14.1); ax.set_ylim(0, 5.4); ax.axis("off")
    g = C["gris"]
    fs = 6.6
    caja(ax, 0.6, 4.2, 2.9, 0.9, "Control de altura", "ec. (35)-(36) + integral", fs=fs)
    caja(ax, 0.6, 2.35, 2.9, 0.9, "Control lateral", "sin sensor de $x$, $y$", fc="white", ec="#9a9a9a", ls="--", col=g, fs=fs)
    caja(ax, 4.3, 2.35, 2.9, 0.9, "Control de actitud", "P + ec. (26) invertida", fs=fs)
    caja(ax, 7.9, 2.35, 2.9, 0.9, "Velocidad angular", "P + ec. (41)", fs=fs)
    caja(ax, 11.2, 2.35, 1.4, 2.75, "Mezclador", "ec. (46)", fc="#dff1f7", ec=C["azul"], fs=fs)
    caja(ax, 13.0, 2.35, 1.0, 2.75, "Dron", fc="#222222", ec="#222222", col="white", fs=fs)
    caja(ax, 4.3, 0.35, 6.5, 0.9, "Estimación", "Mahony (actitud) · Kalman (altura)", fs=fs)
    # mando
    flecha(ax, (3.5, 4.65), (11.2, 4.65)); ax.text(7.35, 4.78, "$u_1$", ha="center", fontsize=8)
    flecha(ax, (3.5, 2.8), (4.3, 2.8), color="#9a9a9a", ls="--")
    ax.text(0.5, 2.0, r"$\phi_d,\ \theta_d$: desde la página web", ha="left", fontsize=6.0, color=g, style="italic")
    flecha(ax, (5.75, 3.75), (5.75, 3.25)); ax.text(5.75, 3.85, r"$\psi_d$", ha="center", fontsize=8)
    flecha(ax, (7.2, 2.8), (7.9, 2.8)); ax.text(7.55, 2.93, r"$\omega_d$", ha="center", fontsize=8)
    flecha(ax, (10.8, 2.8), (11.2, 2.8)); ax.text(11.0, 2.93, r"$\tau$", ha="center", fontsize=8)
    flecha(ax, (12.6, 3.72), (13.0, 3.72)); ax.text(12.8, 3.85, r"$\omega_i$", ha="center", fontsize=8)
    # realimentación
    linea(ax, [13.5, 13.5, 10.85], [2.35, 0.8, 0.8]); flecha(ax, (10.9, 0.8), (10.8, 0.8), color=g)
    ax.text(12.15, 0.5, "IMU · ultrasonido · ToF", ha="center", fontsize=6.0, color=g, style="italic")
    flecha(ax, (5.75, 1.25), (5.75, 2.35), color=g); ax.text(5.9, 1.75, r"$\phi,\ \theta,\ \psi$", fontsize=7, color=g)
    flecha(ax, (9.35, 1.25), (9.35, 2.35), color=g); ax.text(9.5, 1.75, r"$p,\ q,\ r$", fontsize=7, color=g)
    linea(ax, [4.3, 0.2, 0.2], [0.8, 0.8, 4.65]); flecha(ax, (0.2, 4.65), (0.6, 4.65), color=g)
    ax.text(2.25, 0.95, r"$z,\ \dot z$", ha="center", fontsize=7, color=g)
    fig.savefig(os.path.join(OUT, "f_esquema.png"))
    plt.close(fig)


# ----------------------------------------------------------------------------
# 2. MANTENER ALTURA en cuatro vuelos
# ----------------------------------------------------------------------------
VUELOS = [("V10_20261126_174217", "(a) 17:42 · V\\_10"),
          ("V10_20261126_174507", "(b) 17:45 · V\\_10"),
          ("V10b_20261126_183642", "(c) 18:36 · V\\_10b"),
          ("V11_20261126_190936", "(d) 19:09 · V\\_11")]


def fig_hold_versiones():
    fig, axs = plt.subplots(2, 2, figsize=(W2, 3.6))
    print("\nMANTENER ALTURA (muestras con alt_modo = 1):")
    print("vuelo                    dur[s]  us-ref media  rms   max|e|")
    for ax, (tag, nombre) in zip(axs.flat, VUELOS):
        d = cargar(tag)
        t = d["t_vuelo_s"]
        hold = d["alt_modo"] == 1
        fin = t[hold][-1]
        m = t <= fin + 0.02
        us = d["us_m"]
        ax.step(t[m], us[m], where="post", color=C["gris"], lw=0.9, label="altura medida (ultrasonido)")
        ref = np.where(hold, d["alt_ref_m"], np.nan)
        ax.plot(t[m], ref[m], color=C["naranja"], lw=1.6, label="referencia (MANTENER ALTURA)")
        # sombrear tramos en modo
        idx = np.where(hold & m)[0]
        cortes = np.where(np.diff(idx) > 1)[0]
        ini = np.r_[idx[0], idx[cortes + 1]]; fin_ = np.r_[idx[cortes], idx[-1]]
        for a, b in zip(ini, fin_):
            ax.axvspan(t[a], t[b], color="#bfe3f2", alpha=0.45, lw=0)
        ax.set_xlim(0, fin); ax.set_ylim(0, 1.6)
        ax.set_title(nombre.replace("\\_", "_"), loc="left", fontsize=7.2)
        ax.set_ylabel("altura [m]"); ax.set_xlabel("tiempo de vuelo [s]")
        e = (us - d["alt_ref_m"])[hold & np.isfinite(us)]
        dur = hold.sum() / 50.0
        print("%-24s %5.1f   %+6.2f      %5.2f  %5.2f" % (tag, dur, e.mean(), np.sqrt((e ** 2).mean()), np.abs(e).max()))
    h, l = axs[0, 0].get_legend_handles_labels()
    h.append(plt.Rectangle((0, 0), 1, 1, color="#bfe3f2", alpha=0.45)); l.append("MANTENER ALTURA activo")
    fig.legend(h, l, loc="lower center", ncol=3, frameon=False, bbox_to_anchor=(0.5, -0.02))
    fig.tight_layout(rect=(0, 0.05, 1, 1), h_pad=0.8, w_pad=1.0)
    fig.savefig(os.path.join(OUT, "f_hold_versiones.png"))
    plt.close(fig)


# ----------------------------------------------------------------------------
# 3. Actitud en el tramo continuo más largo de dos vuelos
# ----------------------------------------------------------------------------
def tramo_mas_largo(t, gap=0.1):
    br = np.where(np.diff(t) > gap)[0]
    ini = np.r_[0, br + 1]; fin = np.r_[br, len(t) - 1]
    k = np.argmax(t[fin] - t[ini])
    return ini[k], fin[k]


def fig_actitud():
    vuelos = [("V10b_20261126_183216", "18:32 · V_10b"), ("V10_20261126_174507", "17:45 · V_10")]
    ejes = [("roll_deg", r"roll $\phi$ [°]", C["azul"]),
            ("pitch_deg", r"pitch $\theta$ [°]", C["verde"]),
            ("yaw_deg", r"yaw $\psi$ [°]", C["naranja"])]
    datos = []
    for tag, nombre in vuelos:
        d = cargar(tag); a, b = tramo_mas_largo(d["t_vuelo_s"])
        t = d["t_vuelo_s"][a:b + 1]; datos.append((d, a, b, t - t[0], nombre))
    anchos = [max(x[3][-1], 1) for x in datos]
    fig, axs = plt.subplots(3, 2, figsize=(W2, 3.9), sharex="col", gridspec_kw={"width_ratios": anchos})
    print("\nActitud, tramo continuo más largo (RMS respecto de 0°):")
    for j, (d, a, b, t, nombre) in enumerate(datos):
        fila = []
        for i, (col, lab, c) in enumerate(ejes):
            ax = axs[i, j]; v = d[col][a:b + 1]
            ax.plot(t, v, color=c, lw=0.7)
            ax.axhline(0, color=C["negro"], ls="--", lw=0.7)
            rms = np.sqrt(np.mean(v ** 2)); fila.append(rms)
            ax.text(0.99, 0.95, "RMS = %.1f°" % rms, transform=ax.transAxes, ha="right", va="top", fontsize=6.2,
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", ec=C["gris"], lw=0.4))
            if j == 0: ax.set_ylabel(lab)
            if i == 0: ax.set_title("%s · %.0f s continuos" % (nombre, t[-1]), loc="left", fontsize=7.2)
        axs[2, j].set_xlabel("tiempo [s]")
        print("%-24s %5.1f s  roll %.1f  pitch %.1f  yaw %.1f" % (vuelos[j][0], t[-1], *fila))
    fig.align_ylabels(axs[:, 0])
    fig.tight_layout(h_pad=0.4, w_pad=0.8)
    fig.savefig(os.path.join(OUT, "f_actitud.png"))
    plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    fig_esquema()
    fig_hold_versiones()
    fig_actitud()
    print("\nFiguras escritas en", OUT)
