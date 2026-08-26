"""Live plot for the Stage 1 compliance demo, isolated in a SEPARATE PROCESS.

The 400 Hz control loop stays in the main process/thread; it only does a single
non-blocking ``queue.put_nowait()`` every Nth cycle. A spawned child process runs
matplotlib, so rendering can never steal time from -- or interfere with -- the
control loop or its safety shutdown (soft_stop -> motor_release).

matplotlib is imported ONLY inside the child (``_build_figure``), so the headless
Linux robot host never needs it unless plotting is explicitly turned on.

Queue protocol (positional tuple -- the contract), one per pushed cycle::

    (t, q_deg, q0_deg, tau_cmd, tau_clamped, tau_meas, Fx, Fy, px, py, qd, lost)

Plus a single control message ``{"type": "stop"}`` when the run ends. The static
reference data (home pose, limits, friction estimate) is passed once as ``meta``
to the child as a Process arg, so it is available before the first frame.
"""
import multiprocessing as mp
import queue as _queue
import signal

PLOT_HZ = 50.0          # target sample-push rate (control loop is downsampled to this)
WINDOW_SEC = 10.0       # rolling time-series window [s]
TRAIL_LEN = 60          # Cartesian tip-trail length (~1.2 s at 50 Hz)
_FORCE_SCALE = 0.012    # arm-view arrow length: metres drawn per Newton of force


class PlotHandle:
    """Opaque handle returned by :func:`start_plotter`."""
    __slots__ = ("process", "queue", "n")

    def __init__(self, process, queue, n):
        self.process = process
        self.queue = queue
        self.n = n          # push stride: send 1 of every n control cycles


_AVAILABLE = None


def is_available() -> bool:
    """True if matplotlib can be imported (cached). False on a headless host."""
    global _AVAILABLE
    if _AVAILABLE is None:
        try:
            import matplotlib  # noqa: F401
            _AVAILABLE = True
        except Exception:
            _AVAILABLE = False
    return _AVAILABLE


def start_plotter(meta, loop_hz=400.0):
    """Spawn the viewer process. Returns a :class:`PlotHandle`, or None if it
    can't start (no matplotlib / headless) -- callers treat None as "no plot"."""
    if not is_available():
        print("Live plot unavailable (matplotlib not importable) -- running without it.")
        return None
    n = max(1, round(loop_hz / PLOT_HZ))
    try:
        ctx = mp.get_context("spawn")       # macOS default; on Linux avoids
        q = ctx.Queue(maxsize=2000)         # inheriting the open CAN fd via fork
        p = ctx.Process(target=_viewer_main, args=(q, meta), daemon=False)
        p.start()
    except Exception as exc:
        print(f"Live plot failed to start ({exc}) -- running without it.")
        return None
    return PlotHandle(p, q, n)


def push(handle, sample) -> None:
    """Non-blocking push of one sample tuple. Drops on a full queue; never
    blocks the control loop and never raises."""
    if handle is None:
        return
    try:
        handle.queue.put_nowait(sample)
    except Exception:
        pass  # queue.Full (viewer behind) or closed -> drop; CSV log is authoritative


def stop_plotter(handle) -> None:
    """Tell the viewer the run ended (freeze + keep window open). Does NOT join
    or terminate, so the window stays up for inspection."""
    if handle is None:
        return
    try:
        handle.queue.put_nowait({"type": "stop"})
    except Exception:
        pass


def _build_figure(q, meta):
    """Build the 2x2 figure and return ``(fig, update)``.

    Factored out of :func:`_viewer_main` so it can be driven headlessly in a test
    (force the Agg backend, feed a queue, call ``update`` a few times, savefig).
    ``update`` drains ``q`` each call and redraws; on ``{"type": "stop"}`` it
    freezes. matplotlib is imported here, never at module scope.
    """
    import math
    from collections import deque

    import numpy as np
    import matplotlib.pyplot as plt

    q0 = meta["q0_deg"]
    pdx, pdy = meta["p_d"]
    L = meta["L"]
    k_target = meta["k_target"]
    k_ramp = meta.get("K_RAMP_SEC", 2.0)
    tau_clamp = meta.get("TAU_CLAMP_NM", 3.0)
    friction = meta.get("friction_nm", 0.11)
    win = meta.get("window_sec", WINDOW_SEC)

    maxlen = int(math.ceil(win * PLOT_HZ))
    T = deque(maxlen=maxlen)
    Q = deque(maxlen=maxlen)
    TAU_CMD = deque(maxlen=maxlen)
    TAU_CLP = deque(maxlen=maxlen)
    TAU_MEA = deque(maxlen=maxlen)
    trail = deque(maxlen=TRAIL_LEN)
    st = {"frozen": False, "lost": 0, "k_now": 0.0, "err_deg": 0.0,
          "err_mm": 0.0, "qd": 0.0, "tau_cmd": 0.0, "tau_meas": 0.0,
          "px": pdx, "py": pdy, "Fx": 0.0, "Fy": 0.0}

    fig, axs = plt.subplots(2, 2, figsize=(11, 7))
    ax_pos, ax_arm = axs[0]
    ax_tau, ax_eff = axs[1]
    try:
        fig.canvas.manager.set_window_title("Compliance demo -- live")
    except Exception:
        pass

    # (a) position hold ----------------------------------------------------
    ax_pos.set_title("position hold")
    ax_pos.set_ylabel("joint angle [deg]")
    ax_pos.axhline(q0, ls="--", color="k", lw=1.0, label=f"reference q0={q0:+.1f}")
    ax_pos.axhspan(q0 - 2, q0 + 2, color="C0", alpha=0.10)
    (ln_q,) = ax_pos.plot([], [], color="C0", lw=1.6, label="q")
    ax_pos.grid(alpha=0.3)
    ax_pos.legend(loc="upper right", fontsize=8)

    # (b) joint torque -----------------------------------------------------
    ax_tau.set_title("joint torque")
    ax_tau.set_ylabel("torque [N·m]")
    ax_tau.set_xlabel("time [s]")
    ax_tau.axhline(tau_clamp, ls="--", color="tab:red", lw=1.0, label=f"clamp ±{tau_clamp:g}")
    ax_tau.axhline(-tau_clamp, ls="--", color="tab:red", lw=1.0)
    ax_tau.axhspan(-friction, friction, color="gray", alpha=0.25,
                   label=f"friction ±{friction:g}")
    (ln_tc,) = ax_tau.plot([], [], color="C1", lw=2.0, label="τ applied")
    (ln_tu,) = ax_tau.plot([], [], color="C3", lw=0.9, ls=":", label="τ cmd (pre-clamp)")
    (ln_tm,) = ax_tau.plot([], [], color="C2", lw=1.0, label="τ meas")
    ax_tau.set_ylim(-tau_clamp * 1.1, tau_clamp * 1.1)
    ax_tau.grid(alpha=0.3)
    ax_tau.legend(loc="upper right", fontsize=8, ncol=2)

    # (c) Cartesian arm view ----------------------------------------------
    ax_arm.set_title("arm view (link, home, restoring force)")
    ax_arm.set_aspect("equal")
    ax_arm.set_xlim(-L * 1.25, L * 1.25)
    ax_arm.set_ylim(-L * 1.25, L * 1.25)
    _th = np.linspace(0, 2 * np.pi, 200)
    ax_arm.plot(L * np.cos(_th), L * np.sin(_th), ":", color="gray", alpha=0.5)
    ax_arm.plot([0], [0], "ks", ms=5)
    ax_arm.plot([pdx], [pdy], "*", ms=16, color="tab:green", label="home")
    (ln_trail,) = ax_arm.plot([], [], "-", color="C0", lw=1.0, alpha=0.35)
    (ln_link,) = ax_arm.plot([0, pdx], [0, pdy], "o-", color="C0", lw=3.0,
                             ms=7, solid_capstyle="round")
    arrow = ax_arm.quiver([pdx], [pdy], [0.0], [0.0], angles="xy",
                          scale_units="xy", scale=1.0 / _FORCE_SCALE,
                          color="tab:red", width=0.012)
    ax_arm.legend(loc="upper right", fontsize=8)
    ax_arm.grid(alpha=0.3)

    # (d) effort gauge + HUD ----------------------------------------------
    ax_eff.set_title("control effort vs limits")
    ax_eff.set_xlim(0, tau_clamp * 1.05)
    ax_eff.set_ylim(0, 1)
    ax_eff.set_yticks([])
    ax_eff.set_xlabel("|τ cmd| [N·m]")
    gauge = ax_eff.barh([0.78], [0.0], height=0.18, color="tab:gray")[0]
    ax_eff.axvline(friction, color="gray", lw=1.2)
    ax_eff.text(friction, 0.93, " friction", fontsize=7, color="gray", va="center")
    ax_eff.axvline(tau_clamp, color="tab:red", lw=1.2)
    ax_eff.text(tau_clamp, 0.93, " clamp", fontsize=7, color="tab:red", va="center", ha="right")
    hud = ax_eff.text(0.02, 0.55, "", transform=ax_eff.transAxes, family="monospace",
                      fontsize=9, va="top")

    fig.tight_layout()

    def _ramp(t):
        return min(1.0, t / k_ramp) if k_ramp > 0 else 1.0

    def update(_frame):
        if st["frozen"]:
            return []
        got_stop = False
        while True:
            try:
                item = q.get_nowait()
            except _queue.Empty:
                break
            if isinstance(item, dict):
                if item.get("type") == "stop":
                    got_stop = True
                continue
            (t, q_deg, _q0, tau_cmd, tau_clp, tau_meas,
             Fx, Fy, px, py, qd, lost) = item
            T.append(t); Q.append(q_deg)
            TAU_CMD.append(tau_cmd); TAU_CLP.append(tau_clp); TAU_MEA.append(tau_meas)
            trail.append((px, py))
            st.update(lost=lost, k_now=k_target * _ramp(t),
                      err_deg=q_deg - q0, err_mm=math.hypot(px - pdx, py - pdy) * 1000.0,
                      qd=qd, tau_cmd=tau_cmd, tau_meas=tau_meas, px=px, py=py, Fx=Fx, Fy=Fy)

        if got_stop:
            st["frozen"] = True
            ax_pos.set_title("position hold  --  RUN ENDED (close window to exit)")
            try:
                fig.canvas.manager.set_window_title("Compliance demo -- RUN ENDED (close to exit)")
            except Exception:
                pass

        if not T:
            return []

        tnow = T[-1]
        x0 = max(0.0, tnow - win)
        xs = list(T)
        ln_q.set_data(xs, list(Q))
        lo = min(min(Q), q0); hi = max(max(Q), q0)
        mid = 0.5 * (lo + hi); span = max(hi - lo, 10.0)
        ax_pos.set_xlim(x0, max(win, tnow))
        ax_pos.set_ylim(mid - 0.6 * span, mid + 0.6 * span)

        ln_tc.set_data(xs, list(TAU_CLP))
        ln_tu.set_data(xs, list(TAU_CMD))
        ln_tm.set_data(xs, list(TAU_MEA))
        ax_tau.set_xlim(x0, max(win, tnow))

        ln_link.set_data([0, st["px"]], [0, st["py"]])
        if trail:
            tx, ty = zip(*trail)
            ln_trail.set_data(tx, ty)
        arrow.set_offsets([[st["px"], st["py"]]])
        arrow.set_UVC(st["Fx"], st["Fy"])

        val = abs(st["tau_cmd"])
        gauge.set_width(min(val, tau_clamp * 1.05))
        if val < friction:
            gauge.set_color("tab:red")        # spring < static friction -> won't return
        elif val < 0.9 * tau_clamp:
            gauge.set_color("tab:green")
        else:
            gauge.set_color("tab:orange")     # near torque clamp / saturating

        if abs(st["qd"]) > 0.20:
            state_txt = "MOVING"
        elif abs(st["err_deg"]) < 2.0:
            state_txt = "HOLDING"
        elif val < friction:
            state_txt = "PARKED (spring < friction)"
        else:
            state_txt = "RETURNING"
        hud.set_text(
            f"k now : {st['k_now']:6.1f} N/m\n"
            f"error : {st['err_deg']:+6.1f} deg  ({st['err_mm']:5.1f} mm)\n"
            f"|qd|  : {abs(st['qd']):5.2f} rad/s\n"
            f"tau   : cmd {st['tau_cmd']:+.3f}  meas {st['tau_meas']:+.3f} N·m\n"
            f"limits: friction ±{friction:.2f}  clamp ±{tau_clamp:.1f}\n"
            f"lost  : {st['lost']}\n"
            f"state : {state_txt}"
        )
        return []

    return fig, update


def _viewer_main(q, meta):
    """Child-process entry point: build the figure and run the GUI event loop."""
    # Ctrl-C in a terminal hits the whole process group; the viewer must survive
    # so the user can inspect the final plot. Only the control process acts on it.
    try:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    except Exception:
        pass

    import matplotlib
    for backend in ("macosx", "TkAgg", "QtAgg"):
        try:
            matplotlib.use(backend, force=True)
            break
        except Exception:
            continue
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig, update = _build_figure(q, meta)
    # keep a reference so the animation isn't garbage-collected
    _anim = FuncAnimation(fig, update, interval=33, blit=False, cache_frame_data=False)
    plt.show(block=True)
