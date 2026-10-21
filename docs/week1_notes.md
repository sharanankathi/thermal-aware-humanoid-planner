# Week 1 notes: thermal model + scoping

## Decisions locked
- **Actuator:** Kollmorgen TBM(S)-6051-A frameless BLDC, 100:1 strain-wave gear, η = 0.80.
  Continuous joint torque 88 Nm at a 140 °C limit (25 °C ambient). Shoulder/elbow class for an adult-size humanoid.
  Source: Kollmorgen TBM Frameless Motor Selection Guide, KM_SG_00218_RevE_EN, 60-series table.
- **Winding limit:** 140 °C protective stop, 15 °C below the 155 °C rated winding temperature.
- **Robot model:** MuJoCo Menagerie has two humanoids with 7-DOF arms: **Apptronik Apollo** (adult size, matches 88 Nm joints) and **Unitree G1** (29-DOF version). Recommend Apollo with the legs locked; confirm in week 3. No Franka fallback needed.

## Rated-point check (gate) — PASS
| Check | Steady T_w | Datasheet | Rise error |
|---|---|---|---|
| B: current = Ic = 6.54 A (gate) | 161.3 °C | 155 °C | +4.9% |
| A: torque = Tc = 1.08 Nm (info) | 132.6 °C | 155 °C | −17.3% |

Check A misses because the datasheet disagrees with itself: Ic × Kt_hot = 1.151 Nm, not 1.08 Nm. That 6% torque gap is ~13% in loss. Kollmorgen's own ±10% Kt tolerance band (103–198 °C) contains 155 °C. The current check is the right gate because it exercises the model's inputs (R(T), TPR) directly. **Write this up in the paper as-is; do not tune parameters to make A pass.**

## Still provisional (flagged in the YAML)
- R_wh/R_ha split (35/65 of TPR) and both capacitances. Gives τ_w ≈ 2.4 min, τ_h ≈ 21 min. ANSYS housing model replaces these in week 9.
- Gear ratio and efficiency: confirm against a specific Harmonic Drive CSG/CSF size in week 2.

## Findings that change scenario design
1. **The thermal time scale fits a 2-hour shift well.** At 1.15× continuous torque the limit hits at 40 min; at 1.30× at 22 min. Below 1.0× it never hits.
   → Heavy-box scenario must put the hottest joint's *RMS* torque at roughly 1.1–1.4× continuous (97–123 Nm RMS) for baselines to fail inside the shift.
2. **Warm room at 35 °C is a weak stressor.** Continuous torque only drops 88 → 84 Nm (−4.5%), because allowable rise falls 115 → 105 °C and torque scales with its square root. Consider 45 °C (enclosed-cell) as the warm scenario, or report 35 °C honestly as a small effect.

## Reading list (week 1–2), with what to extract
| Paper | Where | Extract |
|---|---|---|
| S. Noda et al., "Online maintaining behavior of high-load and unstable postures based on whole-body load balancing strategy with thermal prediction," IEEE CASE 2014 | IEEE Xplore (USC library) | Do they forecast temperature over a task horizon? Do they choose between limbs, or only redistribute within a posture? Repetitive tasks? |
| Kumagai et al., "Whole body joint load reduction control for high-load tasks of humanoid robot through adapting joint torque limitation based on online joint temperature estimation," IEEE Humanoids 2014 | ieeexplore.ieee.org/abstract/document/7041402 | Their thermal model form and parameters; how torque limits are adapted |
| Jorgensen et al., "Thermal Recovery of Multi-Limbed Robots with Electric Actuators," RA-L 2019 | IEEE / arXiv | Thermal IK cost formulation; first-order model vs your two-node model |
| **New:** "Thermal control of electrical motors for high-power humanoid robots," IEEE/RSJ IROS 2008 (JSK lab, Inaba group) | IEEE Xplore | Not in the plan's prior-work table. Same lab as Noda/Kumagai; check for task-level planning before locking the claim |

Author lists for the two JSK papers came from citation lists; confirm on IEEE Xplore before citing.

## Admin
- [ ] Check arXiv cs.RO endorsement status (arxiv.org/auth/endorse) this week
