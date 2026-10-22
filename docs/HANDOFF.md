# Handoff: v2 (parallel lifting), status 2026-10-21

## Why v2
A reviewer-style check found that a thermally blind rule, "always lift heavy boxes with both arms" (B6),
matched the v1 planner with an exact model. In v1, two-arm lifts cost nothing, so the planner had no
real decision to make. v2 adds **parallel lifting**: each arm carries its own box in the same cycle,
doubling throughput (2,668 boxes/shift max) but heating both arms. That creates the throughput-vs-heat
trade-off the planner exists to manage.

## What changed in the code
- `sim/shift.py`: plan arm `"dual"` = left arm takes this box, right arm the next one, same cycle.
  Assumes the arms don't collide (stated as a limitation).
- `sim/controllers.py`:
  - **B6** always two-arm, **B7** always parallel (thermally blind).
  - **B8** reactive mode switch: parallel until any winding ≥ T_on, two-arm until ≤ T_off, slow +
    pause above T_slow. Variants in the sweep: 120/110 (default), 130/120, 135/125.
  - **Planner**: two-stage lookahead. Hold the fastest action for the next ~27 s window, provided a
    policy exists that finishes the shift safely from the predicted state. Score = boxes over the
    remaining shift; ties go to the cooler option. Plans against 135 °C (5 °C allowance for
    within-cycle temperature ripple). Online loss-gain correction unchanged.
- `thermal/joint_model.py`: `forecast_ttl(..., margin_C=)`.
- `experiments/run_sweep.py`: v2 experiment sets (main / paramerr / ablation / sensitivity).

## Seed-0 results so far (exact model)
| Scenario | B6 two-arm | B8 120/110 | Planner |
|---|---|---|---|
| Heavy, high shelf | 1334 | 1557 | 1640 |
| 35 °C room | 1334 | 1412 | 1522 |
| Right-side conveyor | 1334 | 1531 | 1638 |
| Warm start | 1334 | 1467 | 1588 |
| Mixed | 1334 | 2507 | 2657 |
| Light | 1334 | 2668 | 2668 |

**Open question the sweep must answer:** the planner peaks at 138–139 °C, B8 at 123 °C. B8 tuned to
135/125 got 1545 in the 35 °C room (seed 0), above the planner's 1522. So the planner's edge over the
default reactive rule may mostly be "running hotter". The 20-seed main sweep plus the model-error sweep
(where running near the limit is risky) decide what the paper can claim.

## Next steps
1. **Sharan (Mac):** run the four sweeps (commands in README / below), then send `results/*.csv`.
2. **Claude:** rebuild tables and figures (`make_figures.py` needs v2 labels and colors for
   B6/B7/B8 variants), then rewrite the paper's claims to match. Old v1 results are in
   `results/v1/`; don't mix them with v2.

## Commands (M1 Mac, ~45–60 min total)
```bash
cd thermal-planner && source .venv/bin/activate
git pull            # or apply the patch file
export MENAGERIE_PATH=~/mujoco_menagerie
python -m pytest -q
python experiments/run_sweep.py main
python experiments/run_sweep.py paramerr
python experiments/run_sweep.py sensitivity
python experiments/run_sweep.py ablation
```

## v2 results (Mac sweep, 2026-10-21): the planner does NOT beat a tuned reactive rule
Exact model, 20 runs: B8-135 (parallel until 135 °C, then two-arm until 125 °C) matches or beats the
planner in every scenario (heavy 1673 vs 1646; 35 °C room 1546 vs 1522; conveyor 1640 = 1640;
warm start 1590 vs 1366 with 3 planner stops; mixed 2639 vs 2626). Always two-arm = 1334 everywhere.
Model error ±30%: B8 / B8-130 / B8-135 get 1591–1839 boxes with 0–1 stops; the planner gets 1546–1668
with 2–5 stops (no correction: 8–9 stops). Sensitivity: winding capacity ×0.5 breaks the planner
(8–9/10 stops) and B8-135 (10/10), but not B8-120.
Ablation: lookahead matters inside the planner (constant-policy 1344 vs 1522) but doesn't close the gap.

**Supported findings:** limb allocation (parallel vs two-arm switching) is the thermal lever, worth
15–25% over always-two-arm; posture ≤5%; forecast-based planning adds nothing over a tuned reactive
switch and is more fragile to model error; ~130 °C threshold is the robust sweet spot.
**Obsolete:** every v1 claim (+41%, 1/20 stops), the current paper draft and the portfolio card.
**Decision: A (Oct 21).** Paper rewritten as paper/paper_v2.md (PDF + DOCX), v2 figures from experiments/make_figures_v2.py, portfolio kit v2 (card + 36 s video). Old shared doc marked superseded.
