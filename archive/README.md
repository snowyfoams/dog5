# `archive/` — kept for the record, not maintained

Nothing here is imported by `src/`. It is not on the import path, it is not
covered by the gates, and several files in it will not run in place. It is here
because a repository that only shows the parts that worked is a misleading
account of how the robot was built.

If you are reading the chapters, you do not need this directory. It is
referenced from them where a specific dead end is part of the story.

| path | what it is | active | why it is kept | superseded by |
|---|---|---|---|---|
| `vmc/` | Virtual Model Control: the sim-proven stack and the hardware runner that failed | Jul 2026 | The clearest negative result in the project — see below | `torque_stand/` (ch6) |
| `cartesian_compliance_demo/` | The two-DOF arm Cartesian-compliance demo that predates DOG5 | Feb–Mar 2026 | Where the compliance idea and the first motor library came from | `src/motor/motorbus.py`, ch4 |
| `docs_motor_control/` | `motor_control.tex` (41 KB) + its PDF — an earlier LaTeX write-up of the CAN layer | Mar 2026 | The most detailed prose on the LK/K-TECH protocol; source material for ch1 | `docs/ch1_motor_library.md` |
| `legacy_runners/` | Seven stand/crawl runners with no importers | Jun–Jul 2026 | Each answered one question and was not built on | ch4's `position_stand_crawl/` |
| `legacy_maps/` | `hw_jointmap.py`, `hil_map.json` | Jun 2026 | Two of the four joint maps that were live at once — see ch3 | `dog5_description/dog5_hardware_map.py` |
| `legacy_docs/` | `coordinates.md`, `7.28review.md` | Jul 2026 | `coordinates.md` is the map with left and right swapped | ch3, ch5 |
| `duplicate_tools/` | A diverged second copy of `tools_npz_to_csv.py` | Aug 2026 | Proof the divergence happened; the two produced different columns | `src/tools/tools_npz_to_csv.py` |

## `vmc/` — what actually failed

Worth stating precisely, because the two halves have opposite verdicts.

**The simulation passed.** `test_vmc.py` was run one last time on
2026-08-26 before archiving, and all of gates V1–V4 pass:

```
V1 stand   sag 2.9 mm (<10 mm) · level 0.01 deg (<2 deg)
V2 est     z err 4.7 mm (<30) · |v| err 14.1 mm/s (<50) · roll/pitch 0.11 deg · healthy 1.000
V3 step    did not fall (final z 0.219) · 1800 swing samples · peak tilt 2.30 deg (<10)
V4 ablation, trunk push, estimator live vs frozen:
             live   peak 1.64 deg   rms 0.73   sag  3 mm
             frozen peak 9.87 deg   rms 8.56   sag 33 mm
```

V4 is the interesting one: with the estimator frozen the robot tilts six times
further and sags eleven times more. That is the clearest evidence in the whole
project that the state estimate is doing real work.

**The hardware runner failed.** `vmc_stand_hw.py` could not hold balance rising
from the splayed crouch and was parked on 2026-07-30. The diagnosis is in
`legacy_docs/7.28review.md`. Note that `test_vmc.py` cannot be run from this
directory any more: it needs `dog5_vmc_core`, which was **not** archived —
see below.

**One module was promoted, not archived.** `dog5_vmc_core.py` now lives at
`src/torque_primitives/dog5_vmc_core.py`. It is a live dependency of ch6's
`stance_law.py`, which reuses its `body_wrench`, `grasp_map` and
`distribute_wrench` unchanged on the grounds that they are sim-proven by the
V1–V4 gates above, and of two gate suites. The VMC *math* was never the
problem.

## A wrong claim preserved on purpose

`vmc/stand_hier_hw.py`'s docstring states that the per-joint CAN rate is
~20.8 Hz and concludes that software torque "cannot stabilise the legs". Both
halves are wrong. `MotorBus.slot(rate_hz)` is **per motor**, so 250 Hz means
250 Hz per motor and 3000 frames/s in aggregate — a 12× error, corrected in
commit `c97f1f1` and documented in `src/torque_primitives/torque_params.py`.
The torque track was abandoned on the strength of that number and restarted
once it was fixed; `torque_hold_hw.py` is the experiment that disproved the
verdict. It is left uncorrected here because the mistake is the point.

## Provenance

Both source repositories are tagged `pre-public-reorg-2026-08-26` at the state
this repository was built from:

- `can_motor_control` — the CAN/motor layer, branch `dog5-stand-tools`
- `dog_stand_compliance_control` — everything robot-level, branch `august`

The second was embedded in the first as a gitlink with no `.gitmodules` entry,
so cloning the parent produced an empty directory. That is one of the things
this repository exists to fix.
