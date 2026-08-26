#!/usr/bin/env python3
"""Swing-leg forensics for a trot run npz: FK the feet, measure what the
swing controller actually did, and say which leg is lying.
    cd .../the August track
    python3 data_analysis/tools_swing_analysis.py                  # newest run_*.npz
    python3 data_analysis/tools_swing_analysis.py run_20260824_174511.npz

THE REASONING CHAIN THIS SCRIPT WALKS, in order.  Each section answers one
question, and each question only makes sense once the previous one is
answered -- this is the order to read the output in, and the order to reason
in when a new symptom shows up:

  A. RECONSTRUCT WHAT THE FOOT DID.  The npz logs q (12 joint angles) and
     `planted` (the gait clock's contact mask) every 4 ms sweep.  Joint
     angles + forward kinematics = the foot's path in the trunk frame, no
     extra sensor needed.  Per swing episode it prints:
       - x exc: the largest horizontal excursion from the liftoff point.
         The swing reference commands ZERO x motion (Raibert off, step_xy=0),
         so ANY x here is either dynamic coupling the impedance failed to
         hold, or the trunk rocking underneath the leg.
       - pullback: how far the foot came BACK from that extreme before the
         schedule ended.  Excursion + pullback = the "shoots out, gets
         yanked back" signature; excursion with NO pullback just means the
         foot landed displaced.
       - vx_td: horizontal foot speed at scheduled touchdown.  Hundreds of
         mm/s = scuffing; >1 m/s = the impedance wound up and released.
       - dpitch/(~mm): how much the TRUNK pitched during this swing, and the
         trunk-frame x shift that alone explains for a world-fixed point at
         hip height (z_hip * dpitch).  Compare it against x exc: if it is
         the same size, the "foot motion" is really the body rocking; if x
         exc is 10x bigger, the leg itself is being pushed.
  B. WAS THE ARC EVEN FOLLOWED?  Commanded apex (cfg swing_height) vs the
     apex the FK actually reached, when it peaked (command says phase 0.50),
     and how high the foot still hung when the schedule declared touchdown.
     Reaching ~65% of the apex, late, and landing 15-20 mm in the air is the
     bandwidth fingerprint: the reference outruns kp/kd and the foot spends
     the whole swing catching up.  Fix the DEMAND (swing_height, swing time)
     or the AUTHORITY (kp), not the symptom.
  C. WHO ASKED FOR IMPOSSIBLE TORQUE?  Per swing leg: max |tau_des| per
     joint, the fraction of sweeps any joint sat AT the --tau-max cap, and
     mean |tau_des - tau_cmd| (what the gate refused).  A healthy swing leg
     asks 1-2 Nm; a leg asking 7 Nm against a 4 Nm cap is winding its spring
     up and MUST slingshot when the error finally closes -- that is section
     A's vx_td, explained.  Clipping also breaks the J^T force direction:
     clip one joint of the three and the foot force points somewhere new.
  D. IS IT THE ROBOT OR THE TUNING?  Two discriminators:
       - roll during each leg's swing window: if the sick leg swings exactly
         when roll is deepest, the body may be dropping onto that corner
         (early ground contact mid-swing looks exactly like a wound-up
         impedance).  Swap cfg.LEAD_DIAGONAL to move the schedule against
         the roll and re-run: symptom follows the schedule = dynamics;
         symptom stays on the leg = that leg's hardware.
       - HOLD-stage stance check: mean FK foot position of each leg over the
         quiet four-foot HOLD, against cfg.FOOT_STANCE_BODY.  A leg standing
         >15 mm from nominal has an encoder-zero / geometry problem and
         everything downstream (its swing target foot_xy included) is built
         on that error.  All four within ~10 mm = zeros are fine, look at
         dynamics instead.

WHAT THIS SCRIPT DELIBERATELY DOES NOT DO: touch the robot, modify the log,
or average away per-swing detail.  One bad leg disappears inside a 4-leg
mean -- print per leg, per swing, and let the outlier convict itself.
"""
from __future__ import annotations

import os, sys                                                  # noqa: E401
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import dog5_paths  # noqa: E402,F401  -- every src/ dir onto sys.path

import argparse
import glob
import os
import sys

import numpy as np

# Run from anywhere: the package root is this file's grandparent.
_AUG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from dog5_trot_quasi_static_model import config as cfg     # noqa: E402
import dog5_statics as st                                  # noqa: E402  (config put _TMC on the path)

LEGS = list(cfg.LEGS)


def fk_feet(q_rows):
    """(n, 12) joint angles -> (n, 4, 3) foot sites, trunk frame."""
    out = np.zeros((len(q_rows), 4, 3))
    for k, qr in enumerate(q_rows):
        for i in range(4):
            out[k, i] = st.leg_frames(LEGS[i], qr[3 * i:3 * i + 3])[0]
    return out


def swing_episodes(swinging, min_sweeps=5):
    """[(start, end)) index pairs of contiguous True runs, short ones dropped
    (a 1-2 sweep 'swing' is a contact-mask edge, not a swing)."""
    edges = np.diff(swinging.astype(int))
    starts = np.flatnonzero(edges == 1) + 1
    ends = np.flatnonzero(edges == -1) + 1
    eps = []
    for s in starts:
        e = ends[ends > s]
        if not len(e):
            break
        if e[0] - s >= min_sweeps:
            eps.append((s, int(e[0])))
    return eps


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("npz", nargs="?", default=None,
                    help="run npz (default: newest run_*.npz in cwd)")
    ap.add_argument("--swings", type=int, default=12,
                    help="per-swing rows to print per leg (default 12)")
    args = ap.parse_args()

    f = args.npz or (sorted(glob.glob("run_2026*.npz")) or [None])[-1]
    if f is None:
        sys.exit("no run_*.npz here and none given")
    d = np.load(f, allow_pickle=True)
    t = d["t"]
    q = d["q"]
    stage = d["stage"]
    planted = d["planted"].astype(bool)
    roll = np.degrees(d["roll"])
    pitch = d["pitch"]
    tau_des, tau_cmd = d["tau_des"], d["tau_cmd"]
    cap = float(d["tau_max"])
    h_cmd = float(d["swing_height"])

    print(f"file {f}  sweeps {len(t)}  "
          f"period {float(d['period']):.2f}s duty {float(d['duty']):.2f} "
          f"swing_height {h_cmd*1e3:.0f}mm tau_max {cap} "
          f"raibert_kv {d['raibert_kv']}")

    trot = stage == "TROT"
    if not trot.any():
        sys.exit("no TROT samples in this run")
    idx = np.flatnonzero(trot)
    print(f"TROT window {t[idx[-1]] - t[idx[0]]:.1f}s, {trot.sum()} sweeps")

    # ---- A + B: FK once for the whole TROT window, reuse everywhere -------
    foot = fk_feet(q[idx])
    tp = t[idx]
    pl = planted[idx]
    pi = pitch[idx]
    z_hip = float(d["z_hip"][idx[0]]) if "z_hip" in d.files else np.nan

    for i in range(4):
        eps = swing_episodes(~pl[:, i])
        print(f"\n== {LEGS[i]}: {len(eps)} swings "
              f"(x exc / pullback / vx_td: the shoot-out-and-yank signature; "
              f"~mm: what trunk pitch alone explains) ==")
        for n, (s, e) in enumerate(eps[:args.swings]):
            x = foot[s:e, i, 0]
            z = foot[s:e, i, 2]
            x0, z0 = foot[s - 1, i, 0], foot[s - 1, i, 2]
            dx = x - x0
            j = int(np.argmax(np.abs(dx)))
            back = dx[j] - dx[-1]
            dur = tp[e - 1] - tp[s]
            apex = z.max() - z0
            apex_at = (np.argmax(z)) / max(1, (e - s) - 1)
            vx_td = ((x[-1] - x[-3]) / (tp[s + len(x) - 1] - tp[s + len(x) - 3])
                     if e - s > 3 else np.nan)
            dp = pi[s:e] - pi[s]
            dp_m = dp[int(np.argmax(np.abs(dp)))]
            print(f"  sw{n:2d}: {dur*1e3:4.0f}ms "
                  f"apex {apex*1e3:4.0f}/{h_cmd*1e3:.0f}mm@{apex_at:.2f} "
                  f"land z {1e3*(z[-1]-z0):+5.1f}mm | "
                  f"x exc {dx[j]*1e3:+6.1f}mm@{100*j/(e-s):3.0f}% "
                  f"pull {back*1e3:+6.1f}mm land {dx[-1]*1e3:+6.1f}mm "
                  f"vx_td {vx_td*1e3:+5.0f}mm/s | "
                  f"dpitch {np.degrees(dp_m):+5.2f}deg"
                  f" (~{z_hip*dp_m*1e3:+4.1f}mm)")

    # ---- C: torque demand vs the cap, per swing leg -----------------------
    print(f"\n-- torque asked vs granted during swing (cap {cap} Nm) --")
    for i in range(4):
        m = trot & ~planted[:, i]
        td = tau_des[m][:, 3 * i:3 * i + 3]
        tc = tau_cmd[m][:, 3 * i:3 * i + 3]
        if not td.size:
            continue
        at_cap = np.mean(np.any(np.abs(td) >= cap - 1e-9, axis=1))
        print(f"  {LEGS[i]}: max|tau_des| [abd hip knee] = "
              f"{np.abs(td).max(axis=0).round(2)}  "
              f"sweeps at cap {at_cap:6.2%}  "
              f"mean|des-cmd| {np.abs(td - tc).mean():.3f} Nm")

    # ---- D1: roll during each leg's swing window --------------------------
    print("\n-- roll while each leg swings (deep roll during one leg's "
          "swing = body dropping onto that corner) --")
    for i in range(4):
        m = trot & ~planted[:, i]
        print(f"  {LEGS[i]}: mean {roll[m].mean():+5.2f} deg  "
              f"range [{roll[m].min():+5.1f}, {roll[m].max():+5.1f}]")
    print(f"  all TROT: mean {roll[trot].mean():+5.2f} deg  "
          f"range [{roll[trot].min():+5.1f}, {roll[trot].max():+5.1f}]")

    # ---- D2: HOLD stance check against nominal ----------------------------
    hold = np.flatnonzero(stage == "HOLD")
    if len(hold) > 20:
        sel = hold[len(hold) // 2::max(1, len(hold) // 200)][:200]
        feet = fk_feet(q[sel]).mean(axis=0)
        print("\n-- HOLD stance vs nominal (a leg >15 mm off has a zero/"
              "geometry problem; its swing inherits it) --")
        for i, leg in enumerate(LEGS):
            e = (feet[i] - cfg.FOOT_STANCE_BODY[i]) * 1e3
            print(f"  {leg}: FK [{feet[i, 0]*1e3:+7.1f} {feet[i, 1]*1e3:+7.1f}"
                  f" {feet[i, 2]*1e3:+7.1f}] mm   off nominal "
                  f"[{e[0]:+5.1f} {e[1]:+5.1f} {e[2]:+5.1f}] mm")
    else:
        print("\n-- no usable HOLD stretch in this run; stance check skipped --")


if __name__ == "__main__":
    main()
