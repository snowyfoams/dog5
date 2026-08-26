# `data/` — run logs (not tracked)

This repository ships **no raw experiment data**. The ~300 MB of `run_*.npz`
that the work produced stayed behind in the private archive repositories; the
chapter documents quote the measured numbers instead, with the run they came
from named each time.

Everything in this directory except this file is gitignored.

## Nothing here is required

Every offline gate runs green with this directory empty:

```
python3 src/selftest/test_all.py            # 504 gates, 14 suites
python3 src/state_estimator/test_estimator.py
python3 src/ekf_closeout/test_closeout.py
python3 src/dog5_description/check_dog5_kinematics.py
```

The two gates that *can* use a log check for it first and print `[skip]`
when it is absent. Nothing errors.

## What to drop in, if you want the log-backed checks too

| path | what it is | unlocks |
|---|---|---|
| `data/ekf/stand.npz` | crouch→stand with contacts off, the run the EKF z-scale argument rests on | `test_closeout.py --logs` static case; `replay_full.py --static` |
| `data/ekf/rest.npz` | motionless robot, motors powered | `hw_replay.py --static` |
| `data/crawl/walk_0729_1748.npz` | one position-mode crawl cycle, 2026-07-29 | `test_closeout.py --logs` gait case; `hw_replay.py --gait` |
| `data/torque_trot/t1.npz` `t2.npz` `t3.npz` `t5.npz` | the four torque-mode trot runs of 2026-08-18 | `tools_npz_to_csv.py`, `tools_tau_audit.py`, `tools_swing_analysis.py` |

The four `tN` logs are the ones [ch6](../docs/ch6_quasi_dynamic_control.md)
draws its conclusions from, and `t2` in particular is the run where the tilt
trip failed to fire. They live in the private
`dog_stand_compliance_control` repository under
`Aug_trotinplace_closedloop/data_analysis/raw_npz/`, at tag
`pre-public-reorg-2026-08-26`.

## Reading a log without a robot

`tools_npz_to_csv.py` turns any run into two CSVs — one row per 4 ms sweep with
110 named columns, plus a `_meta.csv` of run scalars — which open in anything:

```
python3 src/tools/tools_npz_to_csv.py data/torque_trot/t1.npz --outdir out
```

Neither the robot, the CAN bus, nor MuJoCo is needed for that.
