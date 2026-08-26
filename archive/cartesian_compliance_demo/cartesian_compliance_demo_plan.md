# Implementation Plan — Cartesian Compliance Demo on a 2-DOF Planar Manipulator

**Target executor:** Claude Code (low-level hardware framework already prepared)
**Secondary purpose:** a staged, observable build so the author can learn impedance control by watching each layer behave.

---

## 0. Scope and Hardware Assumptions

| Item | Value / Assumption |
|---|---|
| Codebase | New controller mode added on top of the existing **`624_Ver2_New_Jump_Controller`** project. The 2-DOF mechanism is the jumping leg (thigh + shank) laid flat on a table as a horizontal planar testbed. |
| Mechanism | 2-DOF serial planar arm, revolute joints $q = [q_1, q_2]$ |
| Link lengths | $l_1 = 0.12\ \mathrm{m}$ (proximal / thigh), $l_2 = 0.132\ \mathrm{m}$ (distal / shank) — from `config.py` |
| Actuators | LKMTECH **MG5010E-i10 V3** at each joint: integrated-drive QDD, ~1:10 planetary reduction, dual encoder (18-bit motor + 14-bit output), torque/speed/position loops over CAN/RS485. **Rated torque 13 N·m, peak 25 N·m** per joint. |
| Actuation mode | **Torque-controlled** (current/torque loop; commanded $\tau$ realized as motor current). |
| Plane | **Horizontal** — joint axes vertical, links sweep the table plane, so gravity is perpendicular to motion and $g(q) = 0$. |
| Force sensing | **None required in the control loop.** Compliance is generated from position feedback. Motor current (via `torque_gain = 206.04` in `config.py`) is used only for *offline validation* (Stage 4). |
| Low-level layer | Already implemented in the repo — bind to the existing modules rather than re-writing (see §1). |

Three consequences of this specific hardware worth stating up front:

1. **The full dynamic model is not needed for this demo.** `config.py` carries link masses ($m_1, m_2, m_b$), CoM positions ($r_1, r_2$), and inertias ($J_1, J_2$) for the jumping dynamics. Level-1 Cartesian stiffness control in the horizontal plane requires *only the Jacobian* $J(q)$, which depends on $l_1, l_2$ alone. No mass, CoM, or inertia value enters the control law, and $g(q) = 0$. Leave `dynamics.py` out of the loop.
2. **Torque headroom is large; the binding constraint on stiffness is loop rate, not the motor.** With reach $\approx l_1 + l_2 = 0.252\ \mathrm{m}$, even a stiff-direction force of several newtons maps to roughly 1 N·m of joint torque — far below the 13 N·m rating. Maximum achievable stiffness is therefore limited by control-loop rate and sensor noise (discrete-time stability), not by saturating the actuator.
3. **Dual output-side encoders make the displacement measurement clean.** Because each joint has an encoder after the gearbox, $q$ (and therefore the end-effector displacement used in Stage 4) is not corrupted by gear backlash. The non-ideal effect that *will* show up is friction in the current→torque channel (see §7).

The horizontal-plane assumption is the single biggest simplifier: the control law collapses to $\tau = J^\top F$ with no dynamic (gravity/Coriolis) compensation. Do **not** add gravity terms.

---

## 1. Interface Contract — Bind to the Existing Repo Modules

The low-level layer already exists. **Before writing any control code, read these files and reuse their function signatures** rather than inventing new ones:

- **`sensor.py`** — joint state: read $q$ (and $\dot q$ if exposed). Confirm units (rad / rad·s⁻¹) and sign/zero conventions.
- **`motor_library.py`** — torque command (sends $\tau$ as motor current) and current/torque readback. This is both the actuation path and the Stage-4 force-measurement path.
- **`config.py`** — `l1, l2`, `torque_gain`, and any existing limits. Add the new `K_c` / `D_c` presets here.
- **`protect.py`** — existing protection (torque/limit guards built for the jumping robot). Reuse it for the safety layer in §6 instead of writing a parallel one; confirm what torque limit it currently enforces.
- **`a_plot_log.py`** + `log/` — existing logging/plotting. Extend it for the Stage-4 force–displacement plot.
- **`finite_state_machine.py` / `main.py`** — the existing loop and mode structure. Add the Cartesian-compliance behavior as a new mode here; do not fork a separate program.

The control layer needs exactly four operations from the above, whatever their actual names:

```
read q, q_dot      # from sensor.py      (rad, rad/s)
send tau           # via motor_library   (N·m → current)
read tau_meas      # via motor_library   (N·m, from current; Stage 4 only)
loop rate          # from main.py / FSM  (Hz)
```

If $\dot q$ is not exposed directly, finite-difference $q$ and low-pass filter it (noisy velocity directly corrupts the damping term and causes visible chatter — the dual encoder gives clean $q$, so a light filter suffices).

---

## 2. Mathematical Foundation

**Forward kinematics** (end-effector position, link lengths $l_1, l_2$):

$$x = l_1\cos q_1 + l_2\cos(q_1+q_2), \qquad y = l_1\sin q_1 + l_2\sin(q_1+q_2)$$

**Jacobian** (maps joint rates to task-space velocity, $\dot{p} = J\dot{q}$):

$$J(q) = \begin{bmatrix} -l_1 s_1 - l_2 s_{12} & -l_2 s_{12} \\ \;\;l_1 c_1 + l_2 c_{12} & \;\;l_2 c_{12} \end{bmatrix}, \quad s_1=\sin q_1,\; s_{12}=\sin(q_1+q_2),\; \text{etc.}$$

**Singularity locus:** $\det J = l_1 l_2 \sin q_2$, which vanishes at $q_2 = 0$ or $q_2 = \pi$ (arm fully extended or folded). The entire demo must operate away from these configurations (see §6).

**Control law — Cartesian stiffness control (Level-1 impedance / task-space PD):**

$$F = -K_c\,(p - p_d) \;-\; D_c\,\dot{p}, \qquad \dot{p} = J\dot{q}$$

$$\boxed{\;\tau = J^\top F\;}$$

- $p_d$ — task-space equilibrium (set point).
- $K_c$ — $2\times2$ Cartesian stiffness matrix (the design object that produces directional compliance).
- $D_c$ — $2\times2$ Cartesian damping matrix.

**Why this version, not full impedance with inertia shaping.** The full formulation $M_d\ddot{p} + D\dot{p} + Kp = F_{ext}$ shapes the *apparent inertia* and therefore requires (a) a measurement/estimate of the external force $F_{ext}$ and (b) the task-space inertia $\Lambda = (J M^{-1} J^\top)^{-1}$, which depends on an accurate dynamic model. For a quasi-static push-and-release demo this is over-engineering and adds fragility. Level-1 control leaves the apparent inertia equal to the arm's natural inertia but renders the **stiffness and damping exactly as designed** — sufficient and robust for demonstrating compliance. (If asked why inertia is not shaped: the demonstration is quasi-static, so the inertia term does not dominate, and skipping it removes the dependence on force estimation.)

---

## 3. The Core of the Demo — Anisotropic (Directional) Stiffness

A uniformly soft end-effector is **not** a convincing demonstration of *Cartesian* compliance: joint-space compliance can produce a similar "soft everywhere" feel. The property that is unique to task-space stiffness is **direction selectivity** — soft along one Cartesian axis, stiff along another:

$$K_c = \begin{bmatrix} k_x & 0 \\ 0 & k_y \end{bmatrix}, \qquad k_x \ll k_y$$

Pushing the end-effector along the soft axis lets it slide; pushing along the stiff axis meets resistance. Same end-effector, opposite "personality" depending on direction — a behavior joint compliance cannot reproduce. **This is the visual answer to "why is this Cartesian and not joint compliance."**

**Optional rigor — rotated stiffness ellipse.** To prove the soft direction can be *any* line (not just an axis), rotate the stiffness:

$$K_c = R(\theta)\,\mathrm{diag}(k_\parallel, k_\perp)\,R(\theta)^\top, \qquad R(\theta) = \begin{bmatrix}\cos\theta & -\sin\theta \\ \sin\theta & \cos\theta\end{bmatrix}$$

This yields a **full, non-diagonal** $K_c$, demonstrating that compliance is genuinely defined in the task frame.

**Numerical starting point (must be re-tuned on hardware):** a stiffness ratio of roughly 10–20× makes the directionality clearly visible — e.g. soft axis $\sim 50\ \mathrm{N/m}$, stiff axis $\sim 800\ \mathrm{N/m}$. The *ratio* matters more than the absolute values.

---

## 4. Control Loop (pseudocode — implement directly)

```
p_d = FK(read_state().q)          # initialize equilibrium at startup pose → no jump
K_c, D_c = build_stiffness(...)   # diagonal or rotated; ramp K from low to target

every cycle (target rate ≥ a few hundred Hz, ideally 500–1000 Hz):
    q, q_dot = read_state()
    p        = FK(q)
    J        = jacobian(q)
    p_dot    = J @ q_dot
    e        = p - p_d
    F        = -K_c @ e - D_c @ p_dot
    tau      = J.T @ F
    tau      = clamp(tau, tau_limit)        # SAFETY (§6)
    if watchdog_tripped(p_dot, tau): tau = 0
    send_torque(tau)
    log(t, q, q_dot, p, e, F, tau, read_torque_meas())   # Stage 4
```

**Loop rate is a stability parameter, not a convenience.** With discrete-time control, a rate that is too low combined with high stiffness drives the virtual spring unstable (oscillation/divergence). For a 2-DOF arm, Python typically sustains 200–500 Hz, which is adequate; move to C++ only if kHz rates are needed.

**Damping selection:** $d_i \approx 2\zeta\sqrt{k_i\, m_{\mathrm{eff}}}$ with $\zeta \approx 0.7\text{–}1$ as a starting estimate, then hand-tune to "push once, no residual bounce." The soft direction tends to look bouncy and usually needs proportionally more damping.

---

## 5. Staged Build (each stage ends in an observable result)

This ordering is deliberate: every stage produces something you can *watch*, so a failure is localized and the underlying behavior is understood before complexity is added.

**Stage 0 — Kinematics + scaffolding.**
Implement `FK`, `jacobian`, `det_j`. Validate the Jacobian by finite-difference against `FK` (this catches sign errors immediately). Wire up the §1 interface and confirm units/signs with a static read. *Observable:* numeric Jacobian matches analytic to tolerance; commanded zero torque holds.

**Stage 1 — Isotropic Cartesian stiffness ("hello world").**
$K_c = k\,I$, $D_c = d\,I$, modest $k$. Set $p_d$ to the startup pose. *Observable:* the arm holds position; pushed, it springs back; released, it returns. This validates the entire pipeline (FK → $J$ → $\tau = J^\top F$ → timing) before any directional behavior. This is also where the key intuition lands: **the spring is virtual — it emerges from position feedback alone, with no force sensor.**

**Stage 2 — Anisotropic stiffness (the demo core).**
$K_c = \mathrm{diag}(k_x, k_y)$, $k_x \ll k_y$. *Observable:* push along the soft axis → slides; push along the stiff axis → resists. The directional contrast *is* the demonstration of Cartesian compliance.

**Stage 3 — Rotated stiffness ellipse (optional).**
Full $K_c$ via $R(\theta)$. *Observable:* the soft direction lies along an arbitrary chosen line. Strengthens the "defined in task space" claim.

**Stage 4 — Logging + force–displacement validation.** (see §7)

**Stage 5 — Stiffness sweep + live tuning.**
Sweep the soft-axis $k$ across low/mid/high and overlay the force–displacement lines (slopes fan out linearly with $k$). Add a live keyboard control to scale $K_c$ at runtime. *Observable:* "triple $K$ now" → the end-effector becomes rigid on the spot; reduce $K$ → it yields to a touch. This makes the physical meaning of the stiffness parameter visible on demand.

---

## 6. Safety (cross-cutting — implement from Stage 1, not at the end)

- **Per-joint torque saturation.** Hard clamp each $\tau_i$ to a fixed limit. Non-negotiable.
- **Watchdog.** If $\|\dot{p}\|$ or $\|\tau\|$ exceeds a threshold, switch to zero torque immediately.
- **Singularity avoidance.** Keep $q_2$ away from $0$ and $\pi$; monitor $\det J = l_1 l_2 \sin q_2$. Near a singularity, $(J^\top)^{-1}$ blows up (the Stage-4 force estimate becomes unreliable) and some Cartesian directions become uncontrollable.
- **Smooth commands.** Ramp $K_c$ from low to target at start-up; never step $p_d$.
- **Workspace / joint-limit guards** on $q$.

---

## 7. Quantitative Validation — Force–Displacement Curve (the part hardest to fake)

Visual push-and-release is not sufficient to support hard questioning. Produce a curve: **end-effector displacement on the horizontal axis, contact force on the vertical axis.** The slope is the realized stiffness; soft axis → shallow slope, stiff axis → steep slope.

**Critical: avoid the circular-validation trap.**

- **Do not** back out force from $F = K_c e$. That reproduces the stiffness you set and verifies nothing.
- **Do** obtain force from an *independent* channel. From motor current — converted to joint torque via `torque_gain = 206.04` in `config.py` (confirm its exact direction/units in `motor_library.py`) — recover the measured joint torque $\tau_{\mathrm{meas}}$, then under quasi-static conditions estimate the external wrench:

$$F_{ext} \approx -\,(J^\top)^{-1}\,\tau_{\mathrm{meas}}$$

Plot this current-derived force against the *encoder-derived* displacement $e$. Force and displacement now come from **two independent sensors**, so the recovered slope $\approx K_c$ is a genuine validation, not a tautology. (Sign bookkeeping is an implementation detail; magnitudes are what matter for the slope.)

**Expect, and proactively report, hysteresis.** Loading and unloading curves will not coincide; the gap is friction in the ~1:10 gearbox of the MG5010E joints. This is not a bug — pointing it out demonstrates understanding of non-ideal effects. Note the asymmetry it creates: the *displacement* axis stays clean (the output-side encoder reads true joint angle despite the gearing), but the *force* axis is degraded, because the motor current also pays for gear friction that never reaches the end-point. The current-based force estimate is therefore least trustworthy in the static-friction (stiction) band and most trustworthy during motion / at larger displacements.

---

## 8. Suggested Module Layout for Claude Code

```
EXISTING (reuse / extend — do not duplicate):
  config.py               # has l1, l2, torque_gain; ADD K_c/D_c presets + compliance limits
  sensor.py               # joint state read (q, q_dot)
  motor_library.py        # torque command (-> current) + current/torque readback
  protect.py              # torque/limit guards -> reuse as the §6 safety layer
  a_plot_log.py           # extend for the force-displacement + stiffness-sweep plots
  main.py / finite_state_machine.py   # add "cartesian_compliance" as a new mode
  dynamics.py             # NOT used by this demo (kept for the jump controller)

NEW (small additions):
  kinematics_planar.py    # fk(q), jacobian(q), det_j(q) for the 2R arm
                          #   (check first whether dynamics.py already exposes FK/Jacobian to reuse)
  cartesian_compliance.py # build_stiffness(diag | rotated); tau = J^T(-K_c e - D_c x_dot)
```

---

## 9. Parameters — Resolved, and What Remains to Confirm

Most of what the original draft flagged is now resolved from `config.py` and the hardware:

- ✅ $l_1 = 0.12$, $l_2 = 0.132$ m — from `config.py`.
- ✅ Current→torque conversion — `torque_gain = 206.04` (confirm exact units/direction in `motor_library.py`).
- ✅ Actuator torque envelope — 13 N·m rated / 25 N·m peak per joint.

Still to confirm, mostly by reading the existing repo:

1. **Torque limit for the safety clamp** — read the value `protect.py` already enforces; set the compliance clamp at or below it (well under the 13 N·m rating).
2. **Achievable closed-loop rate** — read it from `main.py` / the FSM loop.
3. **State-read and torque-command signatures** — confirm names, units, and sign conventions in `sensor.py` and `motor_library.py`.
4. **Velocity source** — does `sensor.py` expose $\dot q$, or differentiate $q$ + filter?

---

## 10. Suggested Live Sequence for the Tran Demonstration

1. **Stage 1 running** — push the end-effector, show uniform spring-back. Establishes that compliance comes from feedback, not a force sensor.
2. **Switch to Stage 2** — push along the soft axis (slides) vs. the stiff axis (resists). This is the "why Cartesian, not joint" moment.
3. **Show the force–displacement curves** (§7) — independent-measurement slopes matching the set $K_c$, with the hysteresis noted as friction.
4. **Live-scale $K$** (§5, Stage 5) — triple it (rigid), shrink it (yielding). Demonstrates command of the parameter's physical meaning.
5. *(Optional)* **Stage 3** rotated ellipse — soft along an arbitrary line.

Each step is the visual answer to a question a careful examiner will ask: directionality answers "why Cartesian"; the independent-measurement curve answers "how do you know the stiffness is real"; live scaling answers "do you understand what $K$ means physically"; and the $J^\top F$ mapping on a hand-computable $2\times2$ Jacobian answers "show me the kinematics."

---

## References

- N. Hogan, "Impedance Control: An Approach to Manipulation: Part I — Theory," *ASME J. Dynamic Systems, Measurement, and Control*, 107(1):1–7, 1985. https://doi.org/10.1115/1.3140702
- N. Hogan, "Impedance Control: An Approach to Manipulation: Part II — Implementation," *ASME J. Dynamic Systems, Measurement, and Control*, 107(1):8–16, 1985. (Derives the feedback law imposing a desired Cartesian impedance at the end-point without solving inverse kinematics — the conceptual basis for §2.) https://doi.org/10.1115/1.3140713
- N. Hogan, "Impedance Control: An Approach to Manipulation: Part III — Applications," *ASME J. Dynamic Systems, Measurement, and Control*, 107(1):17–24, 1985. https://doi.org/10.1115/1.3140701
- B. Siciliano, L. Sciavicco, L. Villani, G. Oriolo, *Robotics: Modelling, Planning and Control*, Springer, 2009 — see the Force Control chapter for the textbook treatment of impedance and compliance control. https://link.springer.com/book/10.1007/978-1-84628-642-1
- C. Ott, *Cartesian Impedance Control of Redundant and Flexible-Joint Robots*, Springer Tracts in Advanced Robotics, vol. 49, 2008. The rigid-body chapter is the directly relevant one for this demo. Book: https://link.springer.com/book/10.1007/978-3-540-69255-3 · Chapter "Cartesian Impedance Control: The Rigid Body Case": https://link.springer.com/chapter/10.1007/978-3-540-69255-3_3
