# =============================================================
# run_compliance.py — 1-DOF Cartesian compliance demo (Stages 0-1)
#
# Bus backend is auto-selected by platform (see demo_config.py):
#   macOS dev Mac  -> PEAK PCAN-USB,   Linux robot host -> SocketCAN can0.
#
# Just run the file and pick a stage when it starts (no flags needed):
#   python run_compliance.py            # interactive menu
#
# ...or pass it on the command line (handy for scripting / the robot host):
#   python run_compliance.py --static            # zero-torque read, confirm units/sign
#   python run_compliance.py --stage 1 --k 80    # ramped virtual spring, gentle
#   python run_compliance.py --stage 1 --duration 30
#
# Conversions (encoder/iq/speed <-> SI) live ONLY here, mirroring
# ../Code_ForCanBusTest/main.py. The control core stays in SI + radians.
# =============================================================

import argparse
import os
import time

import numpy as np

from motor_library import LKMotor
import demo_config as cfg
from kinematics import fk
from compliance import build_Kc, build_Dc, compliance_torque
from safety import clamp, Watchdog, in_joint_limits, k_ramp_scale
import live_plot  # optional live viewer (lazily imports matplotlib only if used)


# ---------- CAN <-> SI conversions (mirror main.py) ----------
def enc_to_rad(enc_continuous) -> float:
    return float(np.deg2rad(enc_continuous * cfg.DIR * cfg.ENCODER_GAIN))


def spd_to_radps(spd_raw) -> float:
    return float(np.deg2rad(spd_raw * cfg.DIR / cfg.VEL_STATE_GAIN))


def iq_to_torque(iq_raw) -> float:
    return float(iq_raw * cfg.DIR / cfg.TORQUE_GAIN)


def torque_to_iq(tau) -> int:
    return int(np.clip(tau * cfg.TORQUE_GAIN * cfg.DIR, -cfg.IQ_SAT, cfg.IQ_SAT))


class EncoderUnwrap:
    """Track a wrap-free encoder count from single-turn readings (0..65535)."""

    HALF = 32768
    FULL = 65536

    def __init__(self, enc0):
        self.prev = enc0
        self.turns = 0

    def update(self, enc) -> int:
        d = enc - self.prev
        if d > self.HALF:
            self.turns -= 1
        elif d < -self.HALF:
            self.turns += 1
        self.prev = enc
        return enc + self.turns * self.FULL


def connect() -> LKMotor:
    m = LKMotor(
        bus_interface=cfg.BUS_INTERFACE,
        bus_channel=cfg.BUS_CHANNEL,
        motor_id=cfg.MOTOR_ID,
        bitrate=cfg.BITRATE,
    )
    m.motor_run()
    print(f"Connected: motor_id={cfg.MOTOR_ID} on {cfg.BUS_CHANNEL}")
    return m


def read_status_retry(m, tries=50):
    """Read status-2, retrying past lost packets (returns the tuple or None)."""
    for _ in range(tries):
        res = m.read_motor_status_2()
        if res is not None:
            return res
    return None


# ---------- modes ----------
def run_static(m, hz=10.0):
    """Command zero torque and print joint angle to confirm units/sign/zero."""
    print("STATIC read (zero torque). Ctrl-C to stop.")
    dt = 1.0 / hz
    enc0 = None
    unwrap = None
    while True:
        res = m.torque_loop_control(0)        # zero torque, returns status-2
        if res is None:
            print("  lost packet")
            time.sleep(dt)
            continue
        _temp, iq_raw, spd_raw, enc = res
        if unwrap is None:
            enc0 = enc
            unwrap = EncoderUnwrap(enc0)
        q = enc_to_rad(unwrap.update(enc))
        qd = spd_to_radps(spd_raw)
        p = fk(q, cfg.L)
        print(
            f"  enc={enc:5d}  q={np.rad2deg(q):+8.3f} deg  "
            f"qd={np.rad2deg(qd):+7.2f} dps  "
            f"p=({p[0]:+.4f},{p[1]:+.4f}) m  tau_meas={iq_to_torque(iq_raw):+.3f} Nm"
        )
        time.sleep(dt)


def run_stage1(m, k_target, duration, plot=False):
    """Ramped isotropic Cartesian spring about the startup pose."""
    # --- init: clean read -> equilibrium (captured once, never stepped) ---
    res = read_status_retry(m)
    if res is None:
        raise RuntimeError("No status response from motor at init.")
    _temp, _iq, _spd, enc0 = res
    unwrap = EncoderUnwrap(enc0)
    q0 = enc_to_rad(unwrap.update(enc0))
    if not in_joint_limits(q0, cfg.Q_LIMITS_RAD):
        raise RuntimeError(f"Startup q0={np.rad2deg(q0):.1f} deg outside joint limits.")
    p_d = fk(q0, cfg.L)
    q0_deg = float(np.rad2deg(q0))
    print(f"Stage 1: q0={q0_deg:+.2f} deg  p_d=({p_d[0]:+.4f},{p_d[1]:+.4f})  "
          f"k_target={k_target} N/m  ramp={cfg.K_RAMP_SEC}s  rate={cfg.LOOP_HZ}Hz")

    # --- optional live viewer (separate process; never blocks this loop) ---
    plotter = None
    if plot:
        meta = {
            "q0_deg": q0_deg,
            "p_d": [float(p_d[0]), float(p_d[1])],
            "L": float(cfg.L),
            "k_target": float(k_target),
            "K_RAMP_SEC": float(cfg.K_RAMP_SEC),
            "TAU_CLAMP_NM": float(cfg.TAU_CLAMP_NM),
            "friction_nm": float(getattr(cfg, "JOINT_FRICTION_NM", 0.11)),
            "window_sec": live_plot.WINDOW_SEC,
            "loop_hz": float(cfg.LOOP_HZ),
        }
        plotter = live_plot.start_plotter(meta, loop_hz=cfg.LOOP_HZ)
    plot_i = 0

    wd = Watchdog(cfg.QDOT_MAX, cfg.WATCHDOG_TAU_MAX)

    # --- logging ---
    log_dir = "log"
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, f"Stage1_log_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    log = open(log_path, "w", encoding="utf-8")
    log.write("t,q_rad,qd_rad,x,y,ex,ey,Fx,Fy,tau_cmd,tau_meas\n")
    print(f"Logging -> {log_path}")

    dt = 1.0 / cfg.LOOP_HZ
    alpha = cfg.VEL_FILTER_ALPHA
    qd_f = 0.0
    tau = 0.0          # applied at the TOP of each cycle (computed last cycle); start 0
    lost = 0
    t0 = time.perf_counter()
    next_t = t0

    try:
        while True:
            # 1) command last cycle's torque; the response IS this cycle's state
            res = m.torque_loop_control(torque_to_iq(tau))
            if res is None:
                lost += 1
                next_t += dt
                _pace(next_t)
                continue
            _temp, iq_raw, spd_raw, enc = res

            q = enc_to_rad(unwrap.update(enc))
            qd_f = alpha * qd_f + (1.0 - alpha) * spd_to_radps(spd_raw)
            tau_meas = iq_to_torque(iq_raw)
            t = time.perf_counter() - t0

            # 2) safety: joint limits
            if not in_joint_limits(q, cfg.Q_LIMITS_RAD):
                print(f"\n[SAFE] joint limit hit: q={np.rad2deg(q):.1f} deg -> zero torque")
                break

            # 3) stiffness ramp (never step) + control law
            s = k_ramp_scale(t, cfg.K_RAMP_SEC)
            Kc = build_Kc(k_target * s)
            Dc = build_Dc(Kc, cfg.ZETA, cfg.M_EFF)
            tau_cmd, diag = compliance_torque(q, qd_f, p_d, Kc, Dc, cfg.L)

            # 4) watchdog (pre-clamp) then hard clamp
            if wd.check(diag["pdot"], tau_cmd):
                print(f"\n[SAFE] watchdog tripped: {wd.reason} -> zero torque")
                break
            tau = clamp(tau_cmd, cfg.TAU_CLAMP_NM)

            # 5) log
            p, e, F = diag["p"], diag["e"], diag["F"]
            log.write(
                f"{t:.4f},{q:.5f},{qd_f:.5f},{p[0]:.5f},{p[1]:.5f},"
                f"{e[0]:.5f},{e[1]:.5f},{F[0]:.4f},{F[1]:.4f},{tau:.4f},{tau_meas:.4f}\n"
            )

            # 6) live plot: one non-blocking push per `plotter.n`-th cycle (~50 Hz)
            if plotter is not None:
                plot_i += 1
                if plot_i % plotter.n == 0:
                    live_plot.push(plotter, (
                        t, np.rad2deg(q), q0_deg,
                        float(tau_cmd), float(tau), float(tau_meas),
                        float(F[0]), float(F[1]),
                        float(p[0]), float(p[1]), float(qd_f), lost,
                    ))

            if duration is not None and t >= duration:
                print(f"\nReached duration {duration}s.")
                break

            next_t += dt
            _pace(next_t)
    finally:
        if lost:
            print(f"(lost packets: {lost})")
        log.close()
        print(f"Log saved: {log_path}")
        # freeze the viewer (window stays open for inspection); no join/terminate
        live_plot.stop_plotter(plotter)


def _pace(next_t):
    """Sleep until next_t (perf_counter clock); skip if already behind."""
    now = time.perf_counter()
    if next_t > now:
        time.sleep(next_t - now)


def soft_stop(m):
    """Graceful wind-down: damp the joint to rest, then leave zero torque.

    Commands a pure viscous damper (tau = -STOP_DAMPING * qd, NO spring) while
    the closed loop is still alive, so a moving joint decelerates smoothly
    instead of coasting/flying the instant it is released. Runs until the joint
    is slow for a few cycles or STOP_TIMEOUT_SEC elapses, then commands zero
    torque. Damping only ever opposes motion, so it is safe after a safety trip
    or a joint-limit stop (it cannot drive the joint further). Never raises.
    """
    kd = cfg.STOP_DAMPING
    dt = 1.0 / cfg.LOOP_HZ
    print("Wind-down: damping joint to rest ...")
    tau = 0.0          # command last cycle's damping torque; response = this state
    qd = 0.0
    settled = 0
    t0 = time.perf_counter()
    next_t = t0
    try:
        while time.perf_counter() - t0 < cfg.STOP_TIMEOUT_SEC:
            res = m.torque_loop_control(torque_to_iq(tau))
            if res is not None:
                _temp, _iq, spd_raw, _enc = res
                qd = spd_to_radps(spd_raw)
                if abs(qd) < cfg.STOP_QDOT:
                    settled += 1
                    if settled >= 5:          # ~5 cycles at rest -> done
                        break
                else:
                    settled = 0
                tau = clamp(-kd * qd, cfg.TAU_CLAMP_NM)   # damper for next cycle
            next_t += dt
            _pace(next_t)
        m.torque_loop_control(0)              # leave the joint at zero torque
        print(f"Wind-down done: |qd|={abs(qd):.2f} rad/s.")
    except Exception as exc:
        print(f"Wind-down skipped ({exc}).")


def _plan_from_args(args):
    """Turn CLI flags into (mode, k, duration, plot). mode is None if none given."""
    if args.static:
        return "static", None, None, False
    if args.stage == 1:
        return "stage1", args.k, args.duration, bool(args.plot)  # None/False -> off
    return None, args.k, args.duration, False


def interactive_plan(plot_default=None):
    """Ask the user which stage to run. Returns (mode, k, duration, plot);
    mode None = quit. plot_default (from --plot/--no-plot) skips the plot prompt."""
    print(f"\n=== Cartesian compliance demo  (motor id={cfg.MOTOR_ID}, "
          f"{cfg.BUS_INTERFACE}:{cfg.BUS_CHANNEL}) ===")
    print("  [0] Static read     - zero torque, just read the joint (safe, no motion)")
    print("  [1] Stage 1 spring  - virtual Cartesian spring (ENERGIZES the motor)")
    print("  [q] Quit")
    choice = input("Select stage > ").strip().lower()

    if choice in ("0", "s", "static"):
        return "static", None, None, False
    if choice in ("1", "stage1"):
        kd = input(f"  stiffness k [N/m] (Enter = {cfg.K_SPRING:g}; gentle ~ 80) > ").strip()
        k = float(kd) if kd else cfg.K_SPRING
        dd = input("  duration [s]      (Enter = 15; 0 = until Ctrl-C) > ").strip()
        duration = float(dd) if dd else 15.0
        if duration == 0:
            duration = None
        if plot_default is None:
            lp = input("  Live plot?        (Enter = yes; n = no) > ").strip().lower()
            plot = lp not in ("n", "no")
        else:
            plot = bool(plot_default)
        dur_txt = "until Ctrl-C" if duration is None else f"{duration:g}s"
        print(f"\n  -> Stage 1: k={k:g} N/m, duration={dur_txt}, live plot {'on' if plot else 'off'}")
        print("     Get ready to push the link: it holds the current pose and springs back.")
        return "stage1", k, duration, plot
    return None, None, None, False


def main():
    ap = argparse.ArgumentParser(description="1-DOF Cartesian compliance demo (Stages 0-1)")
    ap.add_argument("--static", action="store_true", help="zero-torque read to confirm units/sign")
    ap.add_argument("--stage", type=int, default=None, choices=[1], help="run a control stage")
    ap.add_argument("--k", type=float, default=cfg.K_SPRING, help="target stiffness [N/m]")
    ap.add_argument("--duration", type=float, default=None, help="run time [s] (default: until Ctrl-C)")
    ap.add_argument("--plot", action=argparse.BooleanOptionalAction, default=None,
                    help="live plot for Stage 1 (--plot / --no-plot); default: ask interactively")
    args = ap.parse_args()

    # No mode on the command line -> ask the user interactively.
    mode, k, duration, plot = _plan_from_args(args)
    if mode is None:
        mode, k, duration, plot = interactive_plan(plot_default=args.plot)
    if mode is None:
        print("Nothing selected. Bye.")
        return

    m = connect()
    try:
        if mode == "static":
            run_static(m)
        elif mode == "stage1":
            run_stage1(m, k_target=k, duration=duration, plot=plot)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        # graceful wind-down: actively damp the joint to rest before releasing,
        # so a moving joint never coasts/flies when the closed loop is torn down.
        try:
            if mode == "stage1":
                soft_stop(m)
            else:
                m.torque_loop_control(0)
        except Exception:
            pass
        m.motor_release()
        print("Motor released.")


if __name__ == "__main__":
    main()
