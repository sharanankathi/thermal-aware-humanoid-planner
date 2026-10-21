# Simulation stage: decisions and findings

## Setup (locked)
- **Robot:** Apptronik Apollo from MuJoCo Menagerie, floating base removed (pelvis welded), legs/torso/neck at the Menagerie stand pose. 14 arm joints, 7 per arm.
- **Joint torques:** MuJoCo inverse dynamics (`mj_inverse`) along minimum-jerk joint trajectories, with the held box as a payload body at the grasp point (uniform cube, 0.24 m). Constraint forces disabled: free-space arm motion. *Not* forward simulation with a tracking controller; a forward-sim cross-check on one shift is still to do.
- **Cycle:** home → above pick → pick → grasp (0.3 s) → lift → above shelf → place → release (0.3 s) → retreat → home. 5.4 s at full pace; pace options 1.0×, 1.25×, 1.5×; pauses 0–15 s.
- **Work cell:** pick table top at 0.85 m, standard shelf 1.04 m, high shelf 1.42 m (shoulders at 1.37 m). 5 pick slots × 6 place slots, all reachable by left, right and two-arm grasps.

## Actuator sizing — changed from week 1
Week 1 used TBM(S)-6051-A at 100:1 (88 Nm continuous). Apollo's shoulder/elbow limit is 114 Nm, so that motor was oversized: nothing could overheat before exceeding Apollo's torque limits.

**New rule:** every arm joint uses TBM(S)-6013-A, gear ratio chosen so motor peak × N × η ≈ Apollo's joint limit:

| Joint | Ratio | Peak (Nm) | Apollo limit | Continuous @25 °C |
|---|---|---|---|---|
| shoulder aa | 70 | 78 | 78 | 23.6 |
| shoulder ie | 60 | 67 | 67 | 20.2 |
| shoulder fe | 100 | 111 | 114 | 33.7 |
| elbow fe | 100 | 111 | 114 | 33.7 |
| wrist roll | 10 | 11 | 10.6 | 3.4 |
| wrist yaw / pitch | 30 | 33 | 34.2 | 10.1 |

Peak-to-continuous ratio 3.3:1, typical of humanoid actuators. 6025-A and 6051-A at the same ratios = motor-size design study.

## Baseline B3 definition — changed
"Minimize sum of torque squared" with raw torques dumped load onto the 10.6 Nm wrist roll (up to 4× its limit). B3 now minimizes **total copper loss** Σ c_j τ_j², c_j = R/(N η Kt)². That is the physically correct energy objective. All controllers also filter out postures that exceed any Apollo torque limit for the current box.

## Proposed controller (as implemented)
1. **Mode + pace** (re-planned every 5 boxes): candidates = {alternate single arms, two-arm grasp} × 3 paces × 6 pauses. Forecast each candidate's mean joint load forward from the current temperatures; choose the highest-throughput candidate whose forecast keeps every winding under the limit until shift end + 60 s margin.
2. **Arm** (every box): the arm whose hottest joint has more forecast budget.
3. **Posture** (every box): from a 9-posture IK library per target, minimize Σ w_j c_j τ_j² with w_j = ((T_lim − T_amb)/(T_lim − T_j))². Cold robot → identical to energy-optimal; hot joint → load shifted off it.
Mass forecast uses an EMA of recent boxes (the controller does not know future boxes).

## Scenario calibration
Heavy box = 8 kg: single-arm peak torque 93% of Apollo's limit (feasible), B1 stops at 9 min.
At 5 kg every two-arm controller finishes the shift easily. Alternating arms halves duty, so a 5 kg scenario only stresses a one-armed robot.

Scenarios: light (2 kg, std shelf), heavy (8 kg, high), warm room (8 kg, 35 °C), mixed (2/4/6/8 kg, both shelves), right-side conveyor (8 kg, picks only from the right), warm start (8 kg, right arm starts at 110 °C winding / 90 °C housing).

## First results (seed 0) — see results/ for 20 seeds
- Warm room: only the proposed controller finishes at full rate (1334 boxes). B2 stops at 64 min (710), B4 slows to 948. **+41% over reactive derating.**
- Conveyor and warm start: B1/B3 stop within 1–15 min; B4 loses 25–80% throughput; P finishes with the most margin.
- **Risk to address:** blind round-robin (B2) also finishes the heavy, conveyor and warm-start scenarios at seed 0, just hotter (126–133 °C vs 121–124 °C). The ±30% model-error sweep decides whether that margin becomes a win. Report honestly either way.

## Bug found and fixed: posture library (results before this fix are archived, not used)
The first posture optimizer took fixed-size normalized gradient steps in the IK null space,
with no line search. It never converged: "optimized" postures had 2–9× MORE copper loss than
min-norm IK and put the wrist roll at up to 4.8× its torque limit. The feasibility filter
rejected all of them, so every controller silently ran min-norm IK. The first sweep's
results were valid for arm/pace levers only. B3/B5 and the IK ablation were meaningless.
Archived in `results/broken_posture_lib/`.

Fix: constrained SLSQP (`ik.optimize_posture`) from the min-norm solution. Equality on the
grasp point, joint limits, static torque ≤ 85% of limit, and a trust region (±0.6 rad
shoulder/elbow, ±0.3 rad wrist). Without the trust region, the optimizer twists the wrist
roll > 1 rad. That's cheap statically, but moving through it in 1 s needs ~3× the wrist's
torque limit (Apollo models large reflected gear inertia at the wrist).

## Finding: posture redundancy barely relieves the payload-carrying joint
With a working optimizer, the best posture cuts total cycle copper loss by 3% on average
(max 17%) and heat in the hottest joint (shoulder flexion) by ≤ 5% at 8 kg. Shoulder flexion
torque ≈ box weight × horizontal distance from the shoulder axis, and the target fixes that
distance. The redundant DOF only moves the arm's own mass. **For manipulation shifts,
thermal relief must come from load allocation across limbs and from pacing, not from IK.**
This positions the paper against Jorgensen et al. (thermal IK) and Noda et al. (whole-body
load balancing within a posture).

## Finding: an open-loop forecast is not robust; online correction fixes it
Model-error sweep: every thermal parameter of the *plant* scaled by an independent factor in
[0.7, 1.3] (one draw per seed, shared by all joints); the controller keeps datasheet values.
Without correction, the proposed controller stopped in 7–8 of 20 runs per scenario. Every
stop occurred when the plant's loss coefficient R/Kt² was ≥ 15% above datasheet: the
forecast said "safe", so it never slowed down. B4 (reactive) survived some of those runs only
because it reacts to measured temperature.

Fix (`ThermalBudget._adapt`): per-joint copper-loss gain, fitted after every box by comparing
the measured winding rise with the model's prediction from the previous measured state
(damped Newton step on the prediction's sensitivity to the gain, clamped to [0.5, 2.5]).
With an exact model the gain stays exactly 1.000, so the main results are unchanged.
Spot check, heavy shelf: runs that stopped at 16/38/49 min now finish with 999/1184/1149 boxes.
Full rerun: `results/paramerr_p.csv`; old numbers kept as "P-noadapt".

**This is a paper section by itself:** forecast-based thermal planning needs closed-loop
model correction to be safe, and the correction is cheap because winding temperature is
already measured on most actuators.

## Known limitations to state in the paper
- Inverse dynamics, not closed-loop forward simulation.
- Fixed base: no balance or leg loading.
- Grasp is scripted (box rigidly attached at the grasp point); no contact or friction.
- Thermal node split and capacitances provisional until the ANSYS housing model.
- Controller reads both node temperatures; on hardware the housing node would be estimated.
