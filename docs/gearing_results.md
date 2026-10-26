# Gear-ratio study: results (Oct 2026)

**Question.** After the cartridge study showed the heat bottleneck is inside the actuator, how much
does the actuator's own design (gear ratio, and two-speed shifting) matter compared with adding
cooling hardware?

**Read this first: what the baseline is.** "Current gearing" is our own design: Kollmorgen TBM(S)-6013-A
motors with ratios chosen so peak motor torque matches Apollo's published joint limits. It is not
Apollo's real actuator (not public), and not Optimus's or Figure's. Sizing ratio for heat versus
speed is standard actuator practice, so every gain below is relative to a design that was never
optimized for heat. This is a quantified design study, not a discovery.

## Findings
1. **Gear ratio is a larger lever than cooling hardware, relative to our baseline.** Raising each
   arm joint's ratio up to 98% of its motor speed limit (capped at 2x), with air cooling only:
   9,734 / 9,734 / 9,721 boxes per 8 h shift at 25 / 35 / 40 °C, versus 9,011 / 8,370 / 8,024 for
   the baseline gearing with a coolant loop and 2 kg radiator. With the raised gears at a
   pessimistic 70% efficiency, re-gearing only ties the loop and radiator (9,030 / 8,326 / 7,946).
   Fig. `figures/gear_fig1_8h.png`, Table 1 in `results/gearing/tables.md`.
2. **Throughput rises steeply with ratio until joints hit their speed limits.** 0.75x: 993 boxes,
   1x: 1,639, 1.25x: 2,295, 1.5x: 2,668 (task ceiling) per 2 h shift at 25 °C, 80% efficiency.
   Above 1.5x the curve is flat because most joints are already capped by motor speed.
   Fig. `gear_fig3_ratio.png`, Table 3.
3. **Two-speed shifting has a narrow edge.** It only helps joints whose speed limit caps the fixed
   ratio (here the elbow, 1.16x headroom). At 70% gear efficiency, an ideal two-speed joint with a
   2x high gear reaches the 2,668-box ceiling and peaks at 84–97 °C; the best fixed ratio reaches
   2,281–2,470 and peaks at 131 °C. At 80% efficiency both reach the ceiling. A 3x high gear is
   worse than 2x, because reflected rotor inertia grows with ratio squared and adds its own heat.
   Fig. `gear_fig2_twospeed.png`, Table 2. Two-speed robot actuators already exist
   (Girard & Asada, arXiv:2405.16652); their thermal benefit had been argued from I²R, not simulated.
4. **The cost: reflected inertia.** At the speed-limited ratio, shoulder flexion's joint-side rotor
   inertia is 0.32 kg·m², 2.8x Apollo's modeled armature (0.11); a 2x high gear makes it 0.56 (5x).
   This hurts collision safety, impact tolerance and back-drivability, none of which is modeled.
   Table 4.

## Limitations
- Baseline ratios are ours; real humanoid actuators may already sit near the thermally better end.
- Arms only, fixed base, one slow repetitive task (peak joint speeds 2–4 rad/s); faster tasks shrink
  the speed headroom that re-gearing uses.
- Gear efficiency held constant per gear (80% or 70%); real strain-wave efficiency varies with ratio,
  load and temperature. Shifting is ideal (no time, mass or loss): an upper bound.
- Motor speed limit taken as the datasheet's rated-power speed (4,000 rpm at 48 V); voltage-limited
  torque near that speed is not modeled.
- All thermal results inherit the provisional 35/65 winding/housing resistance split, which the
  potting bench test (`docs/bench_potting_plan.md`) measures on a real motor.

## Reproduce
```bash
python experiments/run_gearing.py shift2h    # 90 runs
python experiments/run_gearing.py shift8h    # 54 runs
python experiments/run_gearing.py ratio      # 120 runs + results/gearing/ratios.csv
python experiments/make_figures_gearing.py
```
