# Results summary

Apptronik Apollo (MuJoCo Menagerie), fixed base, 14 arm joints, each driven by a Kollmorgen
TBM(S)-6013-A motor with gearing matched to Apollo's torque limits. 2-hour pick-and-place
shift, 5.4 s nominal cycle (maximum 1,334 boxes). Protective stop when any winding reaches 140 °C.
Full tables: `results/table_main.md`, `table_paramerr.md`, `table_ablation.md`.

## Controllers
| | |
|---|---|
| B1 | Dominant (right) arm only |
| B2 | Round-robin: alternate arms |
| B3 | Nearest arm, copper-loss-optimal posture |
| B4 | Reactive derating: slow down + pause above 120 °C (commercial practice) |
| B5 | Round-robin + copper-loss-optimal posture (strongest thermally blind baseline) |
| **P** | Forecast-based: chooses single arm vs two-arm grasp, arm, posture, and pace/pauses so every winding stays under the limit until shift end; corrects its model online from measured temperatures |

## 1. Exact thermal model (20 runs per cell)
| Scenario | B1 | B2 | B3 | B4 | B5 | **P** |
|---|---|---|---|---|---|---|
| Light boxes (sanity) | 1334 | 1334 | 1334 | 1334 | 1334 | **1334** |
| Heavy, high shelf | 103 ✕ | 1334 | 414 ✕ | 1082 | 1334 | **1334** |
| Heavy, 35 °C room | 81 ✕ | 784 ✕ | 278 ✕ | 946 | 792 ✕ | **1334** |
| Mixed payloads | 1051 (14/20 ✕) | 1334 | 1334 | 1334 | 1334 | **1334** |
| Right-side conveyor | 113 ✕ | 1334 | 113 ✕ | 308 ✕ | 1334 | **1334** |
| Right arm starts warm | 12 ✕ | 1334 | 126 ✕ | 1009 | 1334 | **1334** |

✕ = protective stop in all 20 runs. Peak winding where several finish: P runs 5–12 °C cooler than
B2/B5 (e.g. conveyor 121 vs 133 °C).

**Honest reading:** in the 35 °C room, P is the only controller that finishes at full rate,
**+41% over reactive derating** (1334 vs 946) and +70% over round-robin. In three heavy scenarios,
plain alternation also finishes; there P's advantage is thermal margin, not boxes. That margin
turns into boxes as soon as the model is imperfect (section 2).

## 2. Thermal parameters off by up to ±30% (20 runs, plant ≠ controller model)
| Scenario | B2 | B4 | B5 | P, no correction | **P** |
|---|---|---|---|---|---|
| Heavy, high shelf | 934 (8 ✕) | 1006 (4 ✕) | 934 (8 ✕) | 1034 (7 ✕) | **1256 (1 ✕)** |
| Heavy, 35 °C room | 860 (9 ✕) | 897 (5 ✕) | 861 (9 ✕) | 1011 (8 ✕) | **1204 (1 ✕)** |
| Right-side conveyor | 914 (8 ✕) | 572 (9 ✕) | 914 (8 ✕) | 1028 (7 ✕) | **1256 (1 ✕)** |
| Right arm starts warm | 834 (8 ✕) | 951 (4 ✕) | 834 (8 ✕) | 994 (7 ✕) | **1252 (1 ✕)** |

Mean boxes (number of 20 runs that hit a protective stop).
- Without online correction, the forecast is unsafe whenever the real motors run hotter than
  the datasheet: 7–8 stops in 20.
- Per-joint loss-gain correction from measured winding temperature: **1 stop in 20, +25–40% boxes over the best baseline.**
- The remaining failure is one plant draw (seed 6: copper loss 1.40×, housing resistance 1.13×).
  The single gain fits the fast winding response and underestimates slow housing heat-up.
  **Next step:** a second adapted parameter for the housing node.

## 3. Which lever matters (10 runs per cell)
| Scenario | Full P | Arm/two-arm only | Pacing only | Posture only |
|---|---|---|---|---|
| Heavy, 35 °C room | 1334 | 1334 | 633 | 301 ✕ |
| Right-side conveyor | 1334 | 1334 | 681 (2 ✕ in last 30 s) | 122 ✕ |
| Right arm starts warm | 1334 | 1334 | 723 | 155 ✕ |

- **Load allocation across limbs is the lever.** Alone it recovers all of P's throughput.
- **Pacing alone survives at about half throughput.** It's the fallback when only one arm can do the job.
- **Posture redundancy does almost nothing for payload-carrying joints.** The best feasible
  posture cuts shoulder-flexion heat by ≤ 5% at 8 kg: the box's weight × horizontal reach sets
  that torque, and the redundant DOF only move the arm's own mass. This contrasts with
  thermal-IK work (Jorgensen et al., RA-L 2019) aimed at static postures.
- Safety-margin size (0 vs 60 s vs 10 min) makes no measurable difference.

## Claims the data supports
1. Forecast-based allocation of a repetitive bimanual task across limbs, using a two-node
   winding/housing model, sustains full throughput for a 2-hour shift where reactive derating
   loses 25–77% of throughput.
2. Open-loop thermal forecasts are unsafe under realistic (±30%) parameter error; per-joint
   online loss correction from measured winding temperature restores safety (1/20 vs 7–8/20 stops).
3. For manipulation, kinematic redundancy is a weak thermal lever compared with limb allocation and pacing.

## Limitations (state them in the paper)
- Inverse dynamics on scripted min-jerk trajectories, not closed-loop forward simulation with contact.
- Fixed base: no balance, no leg loading.
- Grasp is a rigid payload attachment; no friction or slip.
- Thermal node split and capacitances provisional until the ANSYS housing model.
- The controller reads both node temperatures; on hardware the housing node would be estimated.
- One motor family (Kollmorgen TBM 60-series), one robot.

## 4. Checks (added)
**Forward simulation** (`experiments/forward_check.py`, `results/forward_check.md`): 1 kHz PID tracking
with box-aware inverse-dynamics feedforward reproduces the inverse-dynamics torques. Copper loss is
within 0.1%, tracking error under 0.15°, and time to 140 °C is unchanged. The 8-node quadrature is exact
to 0.001 Nm RMS. Assumption: the payload mass is known after the grasp. Without it, feedback adds 6–24%
copper loss, and a one-arm 8 kg lift saturates the wrist roll.

**Sensitivity to the provisional node split and capacitances** (`run_sweep.py sensitivity`,
`results/sensitivity.csv`, 10 runs × 6 variants × 2 scenarios). Winding τ ranged 37–148 s and
housing τ 5.4–21.5 min. P finished all 1,334 boxes with 0 stops in every variant. The baselines'
absolute numbers move a lot (B2 in the warm room: 498–1,296 boxes), so quote the gaps as nominal-parameter
results. The ordering holds in every variant.
