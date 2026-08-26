# DOG5 Motor Coordinate Worksheet

Use this sheet with the robot supported and the motors at zero torque. Start
from the DOG5 flat calibration pose (all model joint angles at zero). The model
frame is **+X forward, +Y left, +Z up**.

Move only the listed joint a small amount in the stated model-positive
direction and note which motor responds. Set `direction` to `+1` if the raw
encoder count increases during that motion, or `-1` if it decreases. Use the
shortest count change across the 16-bit encoder wrap.

| Leg | Joint | DOG5 model name | Model-positive motion from the flat zero pose | CAN ID | Direction (`+1`/`-1`) |
|---|---|---|---|:---:|:---:|
| Front left (FL) | Hip abduction | `hip_abd_FL` | Move the whole leg toward the robot's left (outboard). | 10 | ______ |
| Front left (FL) | Hip pitch | `hip_pitch_FL` | Sweep the thigh/foot from forward toward the robot's left. | 11 | ______ |
| Front left (FL) | Knee | `knee_FL` | Hold the thigh; bend the shin from forward toward the robot's left. | 12 | ______ |
| Front right (FR) | Hip abduction | `hip_abd_FR` | Move the whole leg toward the robot's left (inboard). | 7 | ______ |
| Front right (FR) | Hip pitch | `hip_pitch_FR` | Sweep the thigh/foot from forward toward the robot's left. | 8 | ______ |
| Front right (FR) | Knee | `knee_FR` | Hold the thigh; bend the shin from forward toward the robot's left. | 9 | ______ |
| Rear left (RL) | Hip abduction | `hip_abd_RL` | Move the whole leg toward the robot's left (outboard). | 1 | ______ |
| Rear left (RL) | Hip pitch | `hip_pitch_RL` | Sweep the thigh/foot from rearward toward the robot's right. | 2 | ______ |
| Rear left (RL) | Knee | `knee_RL` | Hold the thigh; bend the shin from rearward toward the robot's right. | 3 | ______ |
| Rear right (RR) | Hip abduction | `hip_abd_RR` | Move the whole leg toward the robot's left (inboard). | 4 | ______ |
| Rear right (RR) | Hip pitch | `hip_pitch_RR` | Sweep the thigh/foot from rearward toward the robot's right. | 5 | ______ |
| Rear right (RR) | Knee | `knee_RR` | Hold the thigh; bend the shin from rearward toward the robot's right. | 6 | ______ |

## Encoder-scale measurement

Move one identified joint through a measured output angle. If the raw encoder
crosses from `65535` to `0`, add one wrap; if it crosses from `0` to `65535`,
subtract one wrap.

| Joint / CAN ID | Raw start | Raw end | Wraps | Unwrapped change (counts) | Measured joint change (deg) | Scale (output deg/count) |
|---|---:|---:|---:|---:|---:|---:|
| ____________________ | ______ | ______ | ______ | ______ | ______ | ______ |

`unwrapped change = raw end - raw start + 65536 × wraps`

Recorded encoder scale: ____________________ output deg/count

Equivalent counts/output degree: ____________________
