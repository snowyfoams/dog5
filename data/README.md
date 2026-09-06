# `data/` — run logs (not tracked)

This repository ships **no raw experiment data**. The ~300 MB of `run_*.npz`
the work produced stayed behind in the private archive; the chapter documents
quote the measured numbers instead, naming the run each came from.

Everything in this directory except this file is gitignored.

## Nothing here is required

Every offline gate runs green with this directory empty:

```
python3 src/selftest/test_all.py        # 13 suites, no robot, no CAN, no data
```

## Your own runs land here by default

Every hardware runner logs itself. With no `--log` flag it writes
`run_<date>_<time>.npz` to the current directory at the full 250 Hz —
`tau_cmd`, `tau_meas` and `q_ref` alongside `q`, so the impedance error is
recoverable after the fact.

The `.npz` also embeds `config.snapshot()`: **every constant as imported, plus
`config.py`'s own source text.** The log says what flew, so you never have to
remember. `mv` a run worth keeping to a real name.

## Reading a log without a robot

```
python3 src/tools/tools_npz_to_csv.py run.npz --outdir out
```

Two CSVs: one row per 4 ms sweep with 110 named columns (units always in the
column name), plus a `_meta.csv` of run scalars. They open in anything.
Neither the robot, the CAN bus, nor MuJoCo is needed.

| tool | what it answers |
|---|---|
| `tools_npz_to_csv.py` | give me the whole run as a spreadsheet |
| `tools_tau_audit.py` | `τ_des` vs `τ_cmd` vs `τ_meas` per joint — separates "the control law was wrong" from "a gate clipped it" |
| `tools_swing_analysis.py` | per-swing x excursion, pullback, touchdown speed, torque against cap, roll per swing window |

## The runs chapter 4 draws on

Named here so the numbers in the chapter can be traced, even though the files
are not distributed. They live in the private `dog_stand_compliance_control`
repository under `Aug_trotinplace_closedloop/data_analysis/raw_npz/`, at tag
`pre-public-reorg-2026-08-26`.

| run | what it is |
|---|---|
| `t1`, `t3`, `t5` | torque-mode trot, 2026-08-18. All three hit the 12° tilt e-stop within 0.4–0.8 s of entering TROT |
| `t2` | the same, and **the run where the tilt trip did not fire**: `z` 149 → −53 mm, `Σfz` 136 N. Chapter 4 §9 |
| `s1_kdz40`, `s2_kpz300`, `s3c_att20` | the 2026-08-18 gain ladder — one gain, one run, named in `params.py`'s provenance table |
| `f_ramp_raibert` | 2026-08-20, the run that produced `STEP_INWARD_MAX_M` |
