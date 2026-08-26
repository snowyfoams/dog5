# Chapter 7 — Convex MPC (work in progress)

**Status: OPEN.** The code runs. There is no published hardware result, no
offline gate suite, and no timing guarantee.

Source: [`src/srb_mpc_hw/`](../src/srb_mpc_hw)

---

This chapter documents ~4000 lines that, until this repository was assembled,
appeared in **no markdown file anywhere**. It is included because it is where
the work was heading when the two months ended, and because an undocumented
directory is how code gets lost.

Treat everything here as unvalidated.

## 1. What it is

A port of the MIT Cheetah-style convex MPC (Di Carlo et al., IROS 2018) onto
DOG5.

| module | role |
|---|---|
| `srb_model.py` | the 13-state single-rigid-body model, with an exact discretisation |
| `convex_mpc.py` | the condensed QP over the prediction horizon, plus an in-file ADMM solver |
| `mpc_gait.py` | contact schedule with **measured-touchdown promotion** — a foot becomes stance when it is sensed down, not when the clock says so |
| `mpc_swing.py` | swing trajectory generation |
| `mpc_controller.py` | orchestration; reuses `dynamic_model` and the chapter-6 primitives |
| `mpc_worker.py` | the solver on its own thread with a mailbox, so the CAN sweep never waits on a QP |
| `ekf_feedback.py` | offers the Bloesch EKF to the loop for **velocity only** |
| `mpc_trot_hw.py` | the hardware runner |

The structural bet is the same one chapter 6 made and is worth restating: the
expensive, whole-robot computation runs off the CAN thread, and the sweep only
ever does array arithmetic on fresh telemetry. `mpc_worker.py` is that
boundary.

## 2. Where the state comes from

Deliberately mixed, and it follows chapter 5 §4's division of labour:

- **attitude** — AHRS
- **height** — FK
- **velocity** — the EKF, and *only* velocity

`ekf_feedback.py` names the three states the estimator does not supply. This is
the one place in the project where the Bloesch filter is wired into a
controller rather than run read-only alongside one.

## 3. What is honestly not known

- **No hardware result has been published.** The runner exists; there is no
  logged run in the record that demonstrates it working.
- **No offline gates.** Every other track in this repository has a suite in
  `src/selftest/`; this one has none, so nothing here is covered by the 504
  gates.
- **No solver timing guarantee.** A threaded solve-time figure was measured,
  then **explicitly withdrawn** in commit `b2b115a` as not reproducible. There
  is currently no number for how long a solve takes, which for an MPC running
  against a 10 ms motor watchdog is the number that matters most.
- A contact-detection bug was found and fixed (`9765d1b`) where the detector
  was reading back the swing controller's own intent rather than a measurement
  — the same class of error as chapter 5's mid-air footholds.

## 4. If you pick this up

The obvious order:

1. Write a `test_mpc_*.py` suite in `src/selftest/` — model discretisation
   against finite differences, QP against a known-good solver on random
   instances, gait schedule transitions. Everything else in this repository
   earned its trust that way.
2. Re-measure solve time, on the robot's own hardware, and publish the
   distribution rather than a mean.
3. Only then run it, and only after chapter 6's lead-diagonal experiment —
   because if the roll is a fixed bias rather than a handover asymmetry, MPC
   will inherit it.

## Known limitations / what's next

Everything in §3. This chapter is a signpost, not a result.
