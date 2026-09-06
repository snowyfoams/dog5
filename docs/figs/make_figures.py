#!/usr/bin/env python3
"""Render the two README figures that depend on tunable constants.

    python3 docs/figs/make_figures.py     ->  docs/images/gait_timing.png
                                              docs/images/swing_arc.png

WHY THESE TWO ARE GENERATED AND THE OTHERS ARE NOT.  The coordinate and
kinematics figures in `docs/images/` are geometry: link vectors out of
`dog5.xml`, the IMU relabelling, the four heights.  Move a gain and they are
still true.  These two are not -- they are the gait clock and the swing arc,
and every line in them comes from `dog5_trot_quasi_static_model/config.py` and
from the shipped code that reads it.  So they are drawn BY that code:
`TrotGait` answers the schedule panels and `trot_hw.swing_foot_body` answers
the arc.  Retune `GAIT_PERIOD`, `DUTY`, `CONTACT_RAMP`, `SWING_HEIGHT` or
`WALK_STEP_M`, re-run this, and the README follows.  Nothing here is drawn
from a literal that could quietly go stale.

Needs matplotlib, which is NOT a runtime dependency -- see requirements.txt.
No hardware, no CAN, no IMU.
"""
from __future__ import annotations

import os
import sys

import numpy as np
import matplotlib.pyplot as plt

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, os.path.join(_ROOT, "src"))

from dog5_trot_quasi_static_model import config as cfg          # noqa: E402
from dog5_trot_quasi_static_model import gait as G              # noqa: E402
from dog5_trot_quasi_static_model import trot_hw as T           # noqa: E402
import params as P                                              # noqa: E402

OUT = os.path.join(_ROOT, "docs", "images")

INK, MUT, GRID = "#1F2933", "#64748B", "#DDE3E8"
ACC, BAD, GOOD, BLU = "#0F766E", "#B4400C", "#15803D", "#0B6FA4"
# COLOUR IS BY DIAGONAL, NOT BY LEG.  FL and RR carry the same phase offset
# and so draw the same curve exactly; four colours would suggest four
# schedules where there are two.
LEGS = ("FL", "FR", "RL", "RR")
PAIR = {"FL": BLU, "RR": BLU, "FR": BAD, "RL": BAD}
PAIR_LABEL = {BLU: "FL + RR", BAD: "FR + RL"}


def style():
    plt.rcParams.update({
        "figure.dpi": 170, "savefig.dpi": 170,
        "font.size": 8.5, "axes.labelsize": 8.5, "axes.titlesize": 9.5,
        "axes.titleweight": "bold", "legend.fontsize": 7.5,
        "xtick.labelsize": 7.5, "ytick.labelsize": 7.5,
        "axes.edgecolor": MUT, "axes.labelcolor": INK, "text.color": INK,
        "xtick.color": MUT, "ytick.color": MUT,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.5,
        "axes.spines.top": False, "axes.spines.right": False,
        "figure.facecolor": "white", "savefig.facecolor": "white",
        "savefig.bbox": "tight", "savefig.pad_inches": 0.04,
        "legend.frameon": False,
    })


def save(fig, name):
    path = os.path.join(OUT, name)
    fig.savefig(path)
    plt.close(fig)
    print(f"[fig] {path}")
    return path


# --------------------------------------------------------------- gait timing
def gait_timing():
    """Contact, load ramp and swing phase for four legs over two cycles."""
    g = G.TrotGait()
    g.reset(0.0)
    t = np.linspace(0.0, 2.0 * cfg.GAIT_PERIOD, 3001)
    contact = np.array([g.contact(x) for x in t]).T          # (4, N) bool
    weight = np.array([g.contact_weight(x) for x in t]).T    # (4, N)
    swing = np.array([g.swing_phase(x) for x in t]).T        # (4, N)
    planted = contact.sum(axis=0)

    fig, axes = plt.subplots(4, 1, figsize=(10.4, 7.0), sharex=True,
                             gridspec_kw={"height_ratios": [1.3, 1.0, 1.0, .7],
                                          "hspace": 0.32})
    ax_c, ax_w, ax_s, ax_n = axes

    # every four-foot window, shaded on all panels
    edge = np.diff((planted == 4).astype(int))
    starts = list(t[1:][edge == 1])
    ends = list(t[1:][edge == -1])
    if planted[0] == 4:
        starts.insert(0, t[0])
    if planted[-1] == 4:
        ends.append(t[-1])
    for ax in axes:
        for a, b in zip(starts, ends):
            ax.axvspan(a, b, color=GOOD, alpha=0.10, lw=0, zorder=0)
        for k in (1, 2):
            ax.axvline(k * cfg.GAIT_PERIOD, color=MUT, lw=0.7, ls=":")

    drawn = set()
    for i, leg in enumerate(LEGS):
        y = 3 - i
        on = contact[i]
        d = np.diff(on.astype(int))
        s_ = list(t[1:][d == 1])
        e_ = list(t[1:][d == -1])
        if on[0]:
            s_.insert(0, t[0])
        if on[-1]:
            e_.append(t[-1])
        for a, b in zip(s_, e_):
            ax_c.barh(y, b - a, left=a, height=0.55,
                      color=PAIR[leg], alpha=0.85, zorder=3)
        ax_c.text(-0.02 * t[-1], y, leg, ha="right", va="center",
                  color=PAIR[leg], fontweight="bold")
        # the pair shares one curve to the last decimal, so draw it once
        if PAIR[leg] not in drawn:
            drawn.add(PAIR[leg])
            ax_w.plot(t, weight[i], color=PAIR[leg], lw=1.6,
                      label=PAIR_LABEL[PAIR[leg]])
            ax_s.plot(t, swing[i], color=PAIR[leg], lw=1.6)

    ax_c.set_ylim(-1.35, 3.6)
    ax_c.set_yticks([])
    ax_c.set_title("planted (bar) -- FL+RR and FR+RL take turns; "
                   "green = all four down")
    ax_c.text(0.0, -1.05,
              f"period {cfg.GAIT_PERIOD * 1e3:.0f} ms      "
              f"duty {cfg.DUTY:.2f}      "
              f"offsets {np.array2string(np.asarray(cfg.PHASE_OFFSET))}"
              f"      swing {g.swing_duration * 1e3:.0f} ms",
              color=MUT, fontsize=7.5)

    ax_w.plot(t, weight.sum(axis=0), color=INK, lw=1.0, ls="--",
              label="total")
    ax_w.axhline(1.0, color=GRID, lw=0.8)
    ax_w.set_ylim(-0.05, 5.5)
    ax_w.set_ylabel("contact_weight")
    ax_w.set_title(f"the load ramp -- a smoothstep over CONTACT_RAMP = "
                   f"{cfg.CONTACT_RAMP:.2f} of stance, at BOTH ends, so a "
                   f"foot is unloaded before it lifts")
    ax_w.legend(ncol=3, loc="upper center")

    ax_s.set_ylim(-0.05, 1.05)
    ax_s.set_ylabel("swing_phase")
    ax_s.set_title("swing phase -- 0 at liftoff, 1 at touchdown, "
                   "and exactly 0 while planted")

    ax_n.step(t, planted, where="post", color=INK, lw=1.3)
    ax_n.set_ylim(1.6, 4.4)
    ax_n.set_yticks([2, 3, 4])
    ax_n.set_ylabel("feet down")
    ax_n.set_xlabel("time (s)")
    ax_n.set_xlim(0, t[-1])

    overlap = (cfg.DUTY - 0.5) * cfg.GAIT_PERIOD
    ax_n.set_title(
        f"duty {cfg.DUTY:.2f}, not the textbook 0.50: the overlap "
        f"(duty - 0.5 = {cfg.DUTY - 0.5:.2f}, {overlap * 1e3:.0f} ms) has to "
        f"outlast the ramp (CONTACT_RAMP x duty = "
        f"{cfg.CONTACT_RAMP * cfg.DUTY:.3f})")

    fig.suptitle("The gait clock -- a pure function of t, drawn by "
                 "dog5_trot_quasi_static_model/gait.py",
                 fontweight="bold", fontsize=11, y=0.965)
    return save(fig, "gait_timing.png")


# ----------------------------------------------------------------- swing arc
def swing_arc():
    """The shipped arc, from trot_hw.swing_foot_body, against a sine bump."""
    z_des = P.STAND_HEIGHT                     # floor to trunk BOTTOM
    foot_xy = {0: (cfg.FOOT_STANCE_BODY[0][0], cfg.FOOT_STANCE_BODY[0][1])}
    T_sw = (1.0 - cfg.DUTY) * cfg.GAIT_PERIOD
    h = cfg.SWING_HEIGHT
    step = (cfg.WALK_STEP_M, 0.0)

    s = np.linspace(0.0, 1.0, 1001)
    p = np.array([T.swing_foot_body(0, u, z_des, foot_xy, step_xy=step)[0]
                  for u in s])
    v = np.array([T.swing_foot_body(0, u, z_des, foot_xy, step_xy=step)[1]
                  for u in s]) / T_sw          # per-phase -> per-second
    x0, z0 = p[0, 0], p[0, 2]
    sine = z0 + h * np.sin(np.pi * s)
    sine_v = h * np.pi * np.cos(np.pi * s) / T_sw

    fig, (ax_a, ax_z, ax_v) = plt.subplots(
        1, 3, figsize=(11.6, 3.9),
        gridspec_kw={"width_ratios": [1.05, 1.0, 1.25], "wspace": 0.30})

    ax_a.plot((p[:, 0] - x0) * 1e3, (p[:, 2] - z0) * 1e3, color=ACC, lw=2.0)
    ax_a.plot((s * step[0] - 0.0) * 1e3, (sine - z0) * 1e3, color=BAD,
              lw=1.1, ls="--")
    ax_a.plot([0], [0], "o", color=ACC, ms=5)
    ax_a.plot([step[0] * 1e3], [0], "o", color=ACC, ms=5)
    ax_a.set_aspect("equal", adjustable="datalim")
    ax_a.set_xlabel("forward (mm)")
    ax_a.set_ylabel("up (mm)")
    ax_a.set_title(f"the arc in space\n{cfg.SWING_HEIGHT * 1e3:.0f} mm up, "
                   f"{cfg.WALK_STEP_M * 1e3:.0f} mm along")

    ax_z.plot(s * T_sw * 1e3, (p[:, 2] - z0) * 1e3, color=ACC, lw=2.0,
              label="shipped: smoothstep")
    ax_z.plot(s * T_sw * 1e3, (sine - z0) * 1e3, color=BAD, lw=1.1, ls="--",
              label="sine bump")
    ax_z.set_xlabel("time into swing (ms)")
    ax_z.set_ylabel("height above the print (mm)")
    ax_z.set_title("vertical position")
    ax_z.legend(loc="lower center")

    ax_v.plot(s * T_sw * 1e3, v[:, 2] * 1e3, color=ACC, lw=2.0,
              label="shipped: smoothstep")
    ax_v.plot(s * T_sw * 1e3, sine_v * 1e3, color=BAD, lw=1.1, ls="--",
              label="sine bump")
    ax_v.axhline(0.0, color=MUT, lw=0.8)
    for u, tag in ((0.0, "liftoff"), (0.5, "apex"), (1.0, "touchdown")):
        ax_v.plot([u * T_sw * 1e3], [0.0], "o", color=GOOD, ms=5, zorder=4)
        ax_v.annotate(tag, (u * T_sw * 1e3, 0.0), textcoords="offset points",
                      xytext=(0, 9 if tag == "apex" else 12),
                      ha="center", color=GOOD, fontsize=7.5)
    ax_v.annotate(f"the sine arrives at {abs(sine_v[-1]) * 1e3:.0f} mm/s,\n"
                  f"straight into the floor",
                  (T_sw * 1e3, sine_v[-1] * 1e3), ha="right",
                  textcoords="offset points", xytext=(-16, 64),
                  color=BAD, fontsize=7.5, fontweight="bold",
                  arrowprops=dict(arrowstyle="->", color=BAD, lw=1.0,
                                  shrinkA=2, shrinkB=3))
    ax_v.set_xlabel("time into swing (ms)")
    ax_v.set_ylabel("vertical speed (mm/s)")
    ax_v.set_title("vertical velocity -- the whole argument")
    ax_v.legend(loc="lower left")

    fig.suptitle(
        f"The swing arc as shipped -- trot_hw.swing_foot_body, "
        f"{T_sw * 1e3:.0f} ms of swing.  Two smoothsteps: zero vertical "
        f"speed at liftoff, apex AND touchdown.",
        fontweight="bold", fontsize=10, y=1.04)
    return save(fig, "swing_arc.png")


if __name__ == "__main__":
    style()
    gait_timing()
    swing_arc()
