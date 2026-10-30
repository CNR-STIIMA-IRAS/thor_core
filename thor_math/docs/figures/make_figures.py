#!/usr/bin/env python3
"""Generate the SVG figures of docs/robust_cbf_method.md.

Run from any directory:

    python3 docs/figures/make_figures.py

The curves use the model of cbf.h (reaction for T_r, then braking at a_s,
obstacle with constant acceleration), sampled densely; the parameters are the
values of Tables II and III of the paper unless stated otherwise.
"""

from __future__ import annotations

import os
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Rectangle  # noqa: E402

OUT = Path(__file__).resolve().parent

# Reference palette (light mode) and text tokens.
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#8a8984"
GRID = "#e4e3df"
BLUE = "#2a78d6"
ORANGE = "#eb6834"
AQUA = "#1baf7a"
RED = "#e34948"
SHADE = "#f3d6d3"

plt.rcParams.update({
    "svg.fonttype": "path",
    "svg.hashsalt": "thor_math",
    "font.family": "DejaVu Sans",
    "font.size": 10,
    "axes.edgecolor": MUTED,
    "axes.labelcolor": INK_2,
    "axes.titlecolor": INK,
    "axes.titlesize": 11,
    "axes.titleweight": "bold",
    "axes.titlelocation": "left",
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.8,
    "xtick.color": INK_2,
    "ytick.color": INK_2,
    "lines.linewidth": 2.0,
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
})

# Parameters (paper, Tables II and III).
T_R = 0.15
A_S = 2.5
K = 3.0
SIGMA = (0.02, 0.085, 0.39)
T_F_BAR = 0.3
C0 = 0.06


def distance(tau, d, v_r, v_o, a_o, t_r=T_R, a_s=A_S):
    """d(tau) along the stopping manoeuvre (cbf::minimumDistance, before the stop)."""
    tau = np.asarray(tau, dtype=float)
    value = d + (v_r - v_o) * tau - 0.5 * a_o * tau ** 2
    braking = tau > t_r
    return value + np.where(braking, 0.5 * a_s * (tau - t_r) ** 2, 0.0)


def window(v_r, t_r=T_R, a_s=A_S):
    """End of the window: T_f = T_r - v_r/a_s if approaching, T_r otherwise."""
    return t_r + max(0.0, -v_r / a_s)


def d_min(d, v_r, v_o, a_o, n=4001):
    tau = np.linspace(0.0, window(v_r), n)
    values = distance(tau, d, v_r, v_o, a_o)
    i = int(np.argmin(values))
    return values[i], tau[i]


def save(fig, name):
    fig.savefig(OUT / name, format="svg", bbox_inches="tight", pad_inches=0.15, metadata={"Date": None})
    preview = os.environ.get("THOR_FIGURES_PNG")  # optional PNG copies, to review the layout
    if preview:
        fig.savefig(Path(preview) / name.replace(".svg", ".png"), dpi=110, bbox_inches="tight", pad_inches=0.15)
    plt.close(fig)


# ---------------------------------------------------------------------------
def fig_dmin():
    """d(tau) over reaction and braking, with the minimum and the margin C."""
    # Approaching robot and person, the person decelerating (a_o < 0): the
    # minimum is the stationary point inside the braking phase.
    d0, v_r, v_o, a_o = 0.75, -0.5, 1.0, -4.0
    t_f = window(v_r)
    tau = np.linspace(0.0, t_f, 400)
    dist = distance(tau, d0, v_r, v_o, a_o)
    value, t_star = d_min(d0, v_r, v_o, a_o)
    C = 0.25

    fig, ax = plt.subplots(figsize=(7.2, 3.6))
    ax.axvspan(0.0, T_R, color="#eef4fc", lw=0)
    ax.axvspan(T_R, t_f, color="#fdf0ea", lw=0)
    ax.text(T_R / 2, 0.86, "reazione: v_r costante", ha="center", va="top", color=INK_2, fontsize=9)
    ax.text((T_R + t_f) / 2, 0.86, "frenata: a_s", ha="center", va="top", color=INK_2, fontsize=9)
    ax.plot(tau, dist, color=BLUE)
    ax.axhline(C, color=ORANGE, lw=1.5, ls=(0, (5, 3)))
    ax.text(0.005, C + 0.01, "margine C", color=INK_2, va="bottom", fontsize=9)
    ax.annotate("", xy=(t_star, C), xytext=(t_star, value),
                arrowprops=dict(arrowstyle="<->", color=INK_2, lw=1.2, shrinkA=5, shrinkB=0))
    ax.text(t_star - 0.006, (C + value) / 2, "h = d_min − C", color=INK_2, va="center", ha="right", fontsize=9)
    ax.plot([t_star], [value], "o", ms=8, color=BLUE, mec=SURFACE, mew=2)
    ax.text(t_star, value + 0.03, f"d_min = {value:.2f} m, τ_min = {t_star:.3f} s",
            ha="center", va="bottom", color=INK, fontsize=9)
    for t in (T_R, t_f):
        ax.axvline(t, color=MUTED, lw=0.8)
    ax.set_xticks([0.0, T_R, t_star, t_f], ["0", "T_r", "τ_min", "T_f"])
    ax.set_xlim(0.0, t_f)
    ax.set_ylim(0.0, 0.9)
    ax.set_xlabel("tempo relativo τ [s]")
    ax.set_ylabel("distanza d(τ) [m]")
    ax.set_title("Distanza minima predetta lungo la manovra di arresto")
    save(fig, "fig_dmin.svg")


# ---------------------------------------------------------------------------
def fig_escape():
    """h as a function of d_min for a receding robot: C_escape sets the slope."""
    C = 0.06
    v_r = 0.6
    dm = np.linspace(0.0, 0.14, 300)

    def h(dm, c_e):
        return np.where(dm >= C, dm - C, dm - C + (C - dm) / c_e * T_R * v_r)

    fig, ax = plt.subplots(figsize=(7.2, 3.6))
    ax.axhline(0.0, color=MUTED, lw=0.8)
    ax.axvline(C, color=MUTED, lw=0.8, ls=(0, (2, 2)))
    ax.plot(dm, h(dm, C), color=ORANGE)
    ax.plot(dm, h(dm, 0.25), color=BLUE)
    ax.text(0.005, h(0.005, C) + 0.004, "C_escape = C = 0.06 m\npendenza 1 − T_r v_r / C < 0",
            color=INK, fontsize=9, va="bottom")
    ax.text(0.005, h(0.005, 0.25) - 0.006, "C_escape = 0.25 m\npendenza 1 − T_r v_r / C_escape > 0",
            color=INK, fontsize=9, va="top")
    ax.text(C + 0.002, 0.075, "d_min = C: inizio escape", color=INK_2, fontsize=9, va="top")
    ax.text(0.1, 0.03, "per d_min ≥ C le due curve\ncoincidono: h = d_min − C", color=INK_2, fontsize=9, va="top")
    ax.set_xlim(0.0, 0.14)
    ax.set_ylim(-0.065, 0.085)
    ax.set_xlabel("d_min [m]")
    ax.set_ylabel("h [m]")
    ax.set_title(f"Termine di escape: robot che si allontana a v_r = {v_r} m/s, T_r = {T_R} s")
    save(fig, "fig_escape.svg")


# ---------------------------------------------------------------------------
def fig_geometry():
    """Why a position error widens the bounds of every projection."""
    fig, ax = plt.subplots(figsize=(7.2, 4.0))
    ax.set_aspect("equal")
    ax.axis("off")
    p_o = np.array([0.0, 0.0])
    p_r = np.array([3.2, 0.0])
    b_p = 0.55
    ax.add_patch(Circle(p_o, b_p, facecolor="#fdf0ea", edgecolor=ORANGE, lw=1.5, ls=(0, (4, 3))))
    ax.plot(*p_o, "o", ms=9, color=ORANGE, mec=SURFACE, mew=2)
    ax.plot(*p_r, "o", ms=9, color=BLUE, mec=SURFACE, mew=2)
    ax.text(p_o[0], -0.85, "ostacolo misurato p̂_o\n(sfera di raggio b_p)", ha="center", va="top", color=INK, fontsize=9)
    ax.text(p_r[0], -0.35, "punto robot p_r", ha="center", va="top", color=INK, fontsize=9)

    # Direction u measured and the most rotated one.
    true_o = p_o + np.array([0.0, b_p])
    ax.plot([p_o[0], p_r[0]], [p_o[1], p_r[1]], color=INK_2, lw=1)
    ax.plot([true_o[0], p_r[0]], [true_o[1], p_r[1]], color=INK_2, lw=1, ls=(0, (4, 3)))
    ax.plot(*true_o, "o", ms=6, color=ORANGE)
    ax.text(true_o[0] - 0.1, true_o[1] + 0.12, "p_o vero", ha="right", color=INK_2, fontsize=9)
    ax.add_patch(FancyArrowPatch(p_r, p_r + 0.9 * (p_r - p_o) / np.linalg.norm(p_r - p_o),
                                 arrowstyle="-|>", mutation_scale=14, color=INK, lw=1.6))
    ax.text(p_r[0] + 0.95, 0.12, "u", color=INK, fontsize=11)
    ax.annotate("", xy=(1.9, 0.0), xytext=(1.9, 0.5 * b_p * (3.2 - 1.9) / 3.2 * 2),
                arrowprops=dict(arrowstyle="-", color=MUTED))
    ax.text(1.55, 0.36, "|Δu| ≤ b_p / D", color=INK, fontsize=10)
    ax.annotate("", xy=(p_o[0], -0.25), xytext=(p_r[0], -0.25),
                arrowprops=dict(arrowstyle="<->", color=MUTED, lw=1))
    ax.text(1.6, -0.42, "D = |p_r − p_o|", ha="center", color=INK_2, fontsize=9)

    # Robot velocity: its normal part P v_r changes the projection when u rotates.
    v = np.array([0.4, 1.3])
    ax.add_patch(FancyArrowPatch(p_r, p_r + v, arrowstyle="-|>", mutation_scale=14, color=BLUE, lw=1.6))
    ax.text(p_r[0] + v[0] + 0.08, p_r[1] + v[1], "v_r", color=INK, fontsize=10, va="center")
    ax.plot([p_r[0], p_r[0]], [p_r[1], p_r[1] + v[1]], color=BLUE, lw=1, ls=(0, (2, 2)))
    ax.text(p_r[0] - 0.08, p_r[1] + v[1] / 2, "P v_r", color=INK_2, fontsize=9, ha="right")
    ax.text(4.3, 1.6, "Δ(v_r · u) ≤ |P v_r| · |Δu|\n⇒ b_vr = b_p |P v_r| / D",
            color=INK, fontsize=10, va="top",
            bbox=dict(boxstyle="round,pad=0.4", facecolor=SURFACE, edgecolor=GRID))
    ax.set_xlim(-1.0, 6.6)
    ax.set_ylim(-1.5, 1.9)
    ax.set_title("Un errore di posizione ruota u e allarga i bound di tutte le proiezioni")
    save(fig, "fig_geometry.svg")


# ---------------------------------------------------------------------------
def fig_corner():
    """Level sets of d_min on the (v_o, a_o) plane, box and worst corner."""
    d0, v_r = 0.5, -0.3
    v_o_hat, a_o_hat = 0.4, 0.0
    b_v, b_a = K * SIGMA[1], K * SIGMA[2]
    vo = np.linspace(-0.2, 1.0, 121)
    ao = np.linspace(-2.0, 2.0, 121)
    grid = np.array([[d_min(d0, v_r, x, y, n=600)[0] for x in vo] for y in ao])

    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    ax.grid(False)
    levels = np.arange(0.20, 0.52, 0.04)
    cs = ax.contour(vo, ao, grid, levels=levels, colors=[MUTED], linewidths=1.0)
    ax.clabel(cs, fmt="%.2f m", fontsize=8, colors=INK_2)
    ax.add_patch(Rectangle((v_o_hat - b_v, a_o_hat - b_a), 2 * b_v, 2 * b_a,
                           facecolor="#eef4fc", edgecolor=BLUE, lw=1.5, alpha=0.9))
    ax.plot([v_o_hat], [a_o_hat], "o", ms=8, color=BLUE, mec=SURFACE, mew=2)
    ax.text(v_o_hat - 0.02, a_o_hat + 0.12, "misurato ψ̂", ha="right", color=INK, fontsize=9)
    corner = (v_o_hat + b_v, a_o_hat + b_a)
    ax.plot([corner[0]], [corner[1]], "o", ms=9, color=RED, mec=SURFACE, mew=2)
    ax.annotate("angolo peggiore\n(v̂_o + b_v, â_o + b_a)\nminimo di d_min nella scatola",
                corner, xytext=(corner[0] + 0.08, corner[1] + 0.35), color=INK, fontsize=9,
                arrowprops=dict(arrowstyle="-", color=MUTED, lw=1))
    ax.set_xlabel("v_o: velocità dell'ostacolo verso il robot [m/s]")
    ax.set_ylabel("a_o [m/s²]")
    ax.set_title(f"Curve di livello di d_min (d = {d0} m, v_r = {v_r} m/s) e scatola ±3σ")
    save(fig, "fig_corner.svg")


# ---------------------------------------------------------------------------
def fig_margin():
    """State-dependent margin versus the constant margin of Eq. (28)."""
    s_p, s_v, s_a = SIGMA
    t = np.linspace(0.0, T_F_BAR, 200)
    margin = K * (s_p + s_v * t + 0.5 * s_a * t ** 2)
    eq28 = K * (s_p + s_v * T_F_BAR + 0.5 * s_a * T_F_BAR ** 2)

    fig, ax = plt.subplots(figsize=(7.2, 3.6))
    ax.plot(t, margin, color=BLUE)
    ax.axhline(eq28, color=ORANGE, lw=1.5, ls=(0, (5, 3)))
    ax.fill_between(t, margin, eq28, color="#eef4fc", lw=0)
    ax.text(0.005, eq28 + 0.004, f"eq. (28) con T̄_f = {T_F_BAR} s: {eq28:.3f} m (costante)",
            color=INK, fontsize=9, va="bottom")
    ax.text(0.16, K * (s_p + s_v * 0.16 + 0.5 * s_a * 0.16 ** 2) - 0.01,
            "k (σ_p + σ_v τ + σ_a τ²/2)\nmargine al τ_min vero,\ne crescita della scatola sull'orizzonte",
            color=INK, fontsize=9, va="top")
    ax.text(0.03, 0.12, "margine\nrecuperato", color=INK_2, fontsize=9)
    ax.set_xlim(0.0, T_F_BAR)
    ax.set_ylim(0.0, 0.22)
    ax.set_xlabel("τ_min (o tempo di predizione t) [s]")
    ax.set_ylabel("margine di distanza [m]")
    ax.set_title("Margine di percezione: dipendente dallo stato contro costante (k = 3)")
    save(fig, "fig_margin.svg")


# ---------------------------------------------------------------------------
def fig_robust_idea():
    """The sketch shared in the discussion (panels A-E), with the gamma factor corrected."""
    fig = plt.figure(figsize=(13.0, 9.0))
    fig.suptitle("CBF robusta al rumore di misura su ψ: restringimento di 3σ_ψ lungo il gradiente di h",
                 x=0.04, ha="left", fontsize=13, fontweight="bold", color=INK)
    gs = fig.add_gridspec(3, 2, width_ratios=[1.0, 1.55], height_ratios=[1.0, 1.0, 0.85],
                          hspace=0.45, wspace=0.12)

    # A: linearization.
    ax = fig.add_subplot(gs[0, 0])
    psi = np.linspace(0.0, 10.0, 400)
    hfun = lambda x: 1.2 + 0.9 * np.sin(0.35 * x + 0.2) - 0.02 * x  # noqa: E731
    dh = lambda x: 0.9 * 0.35 * np.cos(0.35 * x + 0.2) - 0.02  # noqa: E731
    ax.plot(psi, hfun(psi), color=INK)
    for x0, sign in ((2.2, -1.0), (7.0, 1.0)):
        s = 1.4
        ax.plot([x0], [hfun(x0)], "o", ms=8, color=AQUA, mec=SURFACE, mew=2)
        xs = np.array([x0 - s, x0 + s])
        ax.plot(xs, hfun(x0) + dh(x0) * (xs - x0), color=INK_2, lw=1)
        xw = x0 + sign * s
        ax.plot([xw], [hfun(x0) + dh(x0) * (xw - x0)], "o", ms=8, color=RED, mec=SURFACE, mew=2)
        ax.annotate("", xy=(x0 - s, 0.1), xytext=(x0 + s, 0.1),
                    arrowprops=dict(arrowstyle="<->", color=AQUA, lw=1.2))
        ax.text(x0, 0.18, "±3σ_ψ", ha="center", color=INK_2, fontsize=8)
        ax.text(x0, hfun(x0) + 0.25, "ψ̂", ha="center", color=INK, fontsize=10)
        label = "∂h/∂ψ > 0\ncaso peggiore ψ̂ − 3σ_ψ" if sign < 0 else "∂h/∂ψ < 0\ncaso peggiore ψ̂ + 3σ_ψ"
        ax.text(x0, 0.45, label, ha="center", color=RED, fontsize=8)
    ax.text(9.9, 2.3, "punto rosso: h − |∂h/∂ψ| 3σ_ψ", ha="right", color=RED, fontsize=8)
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 2.5)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xlabel("ψ")
    ax.set_ylabel("h")
    ax.set_title("A · Caso peggiore di h entro ±3σ_ψ")

    # B: the four sign cases.
    ax = fig.add_subplot(gs[0, 1])
    t = np.linspace(0.0, 10.0, 400)
    h_t = 2.05 + 0.1 * np.sin(0.9 * t) + 0.12 * np.sin(0.4 * t + 1.0)
    psi_t = 0.95 + 0.12 * np.sin(1.1 * t + 0.4) + 0.08 * np.sin(0.5 * t)
    ax.plot(t, h_t, color=INK)
    ax.plot(t, psi_t, color=AQUA)
    ax.text(10.1, h_t[-1], "h(t)", color=INK, va="center")
    ax.text(10.1, psi_t[-1], "ψ̂(t)", color=INK, va="center")
    for tc, lab in ((2.0, "∂h/∂ψ > 0"), (7.0, "∂h/∂ψ < 0")):
        ax.axvline(tc, color=MUTED, lw=0.8, ls=(0, (3, 3)))
        ax.text(tc, 2.55, lab, ha="center", color=AQUA, fontsize=9)
    for tc, above, below in ((2.0, BLUE, RED), (7.0, RED, BLUE)):
        y = np.interp(tc, t, psi_t)
        ax.plot([tc], [y], "o", ms=5, color=AQUA)
        ax.plot([tc], [y + 0.17], "o", ms=8, mfc=SURFACE, mec=above, mew=2)
        ax.plot([tc], [y - 0.17], "o", ms=8, mfc=SURFACE, mec=below, mew=2)
    y1, y2 = np.interp(2.0, t, psi_t), np.interp(7.0, t, psi_t)
    ax.text(2.25, y1 + 0.3, "ψ_vera > ψ̂ ⇒ h_vera > h_mis\nconservativo", color=BLUE, fontsize=8, va="bottom")
    ax.text(2.25, y1 - 0.3, "ψ_vera < ψ̂ ⇒ h_vera < h_mis\nCBF standard non conservativa", color=RED, fontsize=8, va="top")
    ax.text(7.25, y2 + 0.3, "ψ_vera > ψ̂ ⇒ h_vera < h_mis\nCBF standard non conservativa", color=RED, fontsize=8, va="bottom")
    ax.text(7.25, y2 - 0.3, "ψ_vera < ψ̂ ⇒ h_vera > h_mis\nconservativo", color=BLUE, fontsize=8, va="top")
    ax.set_xlim(0, 10.0)
    ax.set_ylim(0.2, 2.7)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xlabel("t")
    ax.set_title("B · I quattro casi di segno: dove la CBF standard non è conservativa")

    # Dimensional check.
    ax = fig.add_subplot(gs[1, 0])
    ax.axis("off")
    ax.add_patch(FancyBboxPatch((0.02, 0.05), 0.96, 0.88, boxstyle="round,pad=0.01",
                                facecolor="#f5f5f3", edgecolor=GRID, transform=ax.transAxes))
    lines = ["Verifica dimensionale", "h [m]     ḣ [m/s]     γ [1/s]", "ψ, σ_ψ [u_ψ]     ∂h/∂ψ [m/u_ψ]",
             "|∂h/∂ψ| 3σ_ψ  [m]", "ḣ ≥ −γ h  ⇒  [m/s] = [1/s][m]  ✓",
             "γ |∂h/∂ψ| 3σ_ψ  [m/s]  ✓", "|∂h/∂ψ| 3σ_ψ  sommato a ḣ:  [m] ≠ [m/s]  ✗"]
    colors = [INK, INK_2, INK_2, INK_2, AQUA, AQUA, RED]
    for i, (line, color) in enumerate(zip(lines, colors)):
        ax.text(0.07, 0.83 - i * 0.115, line, transform=ax.transAxes, color=color,
                fontsize=10 if i else 11, fontweight="bold" if i == 0 else "normal")

    # C: the threshold on dh/dt.
    ax = fig.add_subplot(gs[1, 1])
    t = np.linspace(0.0, 10.0, 400)
    base = -1.0 + 0.25 * np.sin(0.5 * t) + 0.05 * t
    lift = 0.35 + 0.12 * np.sin(0.8 * t + 1.0)
    ax.fill_between(t, -2.0, base, color="#e9e8e5", lw=0)
    ax.fill_between(t, base, base + lift, color=SHADE, lw=0)
    ax.plot(t, base, color=INK)
    ax.plot(t, base + lift, color=RED)
    ax.text(10.05, base[-1], "−γh", color=INK, va="center")
    ax.text(10.05, (base + lift)[-1], "−γh + γ|∂h/∂ψ|3σ_ψ", color=RED, va="center")
    ax.text(4.0, -1.75, "zona vietata (ḣ < −γh)", color=INK_2, fontsize=9)
    ax.text(5.0, np.interp(5.0, t, base + 0.5 * lift), "restringimento robusto", color=RED, fontsize=9, va="center")
    ax.text(0.2, 0.15, "regione ammissibile: ḣ ≥ curva rossa", color=INK_2, fontsize=9)
    ax.set_xlim(0, 10.0)
    ax.set_ylim(-2.0, 0.4)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xlabel("t")
    ax.set_ylabel("ḣ")
    ax.set_title("C · Vincolo su ḣ: la soglia sale di γ|∂h/∂ψ|3σ_ψ")

    # D and E: formulas.
    def formula_box(ax, title, rows):
        ax.axis("off")
        ax.add_patch(FancyBboxPatch((0.01, 0.02), 0.98, 0.94, boxstyle="round,pad=0.01",
                                    facecolor=SURFACE, edgecolor=INK_2, transform=ax.transAxes))
        ax.text(0.04, 0.86, title, transform=ax.transAxes, color=INK, fontsize=11, fontweight="bold")
        for i, (text, color, size) in enumerate(rows):
            ax.text(0.05, 0.68 - i * 0.18, text, transform=ax.transAxes, color=color, fontsize=size, va="center")

    formula_box(fig.add_subplot(gs[2, 0]), "D · Caso scalare (una componente di ψ)", [
        (r"$\dot h \geq -\gamma h$     CBF standard", INK_2, 11),
        (r"$\dot h \geq -\gamma\,(h - |\partial h/\partial\psi|\,3\sigma_\psi)$", INK, 12),
        (r"$\Leftrightarrow\ \dot h \geq -\gamma h + \gamma\,|\partial h/\partial\psi|\,3\sigma_\psi$", RED, 12),
        ("il termine robusto è moltiplicato per γ (correzione dello schizzo)", INK_2, 9),
    ])
    formula_box(fig.add_subplot(gs[2, 1]), "E · Più componenti ψ = (ψ₁, ψ₂, ...)", [
        (r"$\dot h \geq -\gamma h + \gamma \sum_i |\partial h/\partial\psi_i|\,3\sigma_{\psi_i}"
         r" = -\gamma h + 3\gamma\,|\nabla_\psi h|^T \sigma_\psi$", RED, 12),
        (r"variante gaussiana: $\sigma_h = \sqrt{\nabla_\psi h^T\,\Sigma_\psi\,\nabla_\psi h}$,"
         r"  $\dot h \geq -\gamma h + 3\gamma\,\sigma_h$", INK, 11),
        (r"con $\sigma_h \leq \sum_i |\partial h/\partial\psi_i|\,\sigma_{\psi_i}$: la somma è più prudente",
         INK_2, 10),
        ("thor_math usa la forma non linearizzata: h valutata nell'angolo peggiore (§5)", INK_2, 9),
    ])
    save(fig, "fig_robust_idea.svg")


if __name__ == "__main__":
    fig_dmin()
    fig_escape()
    fig_geometry()
    fig_corner()
    fig_margin()
    fig_robust_idea()
    print("figures written to", OUT)
