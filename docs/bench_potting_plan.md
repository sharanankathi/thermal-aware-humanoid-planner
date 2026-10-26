# Bench test: does potting cut a motor's winding-to-housing resistance?

**Why:** every simulation today pointed at one number, the winding-to-housing thermal resistance
`R_wh` inside the motor. Halving it let a coolant loop with no heat sink reach the no-limit
throughput (results/cartridge/joint.csv). This test measures on real hardware how much potting the
windings with thermally conductive epoxy actually reduces `R_wh`.

**Kill criterion (decided before testing):** if potting cuts `R_wh` by less than 25% relative to the
same motor unpotted, while the control motor drifts less than 5%, potting is not the lever.

## Parts (~$150–200)
| Item | Notes |
|---|---|
| 2x identical gimbal BLDC motors, 5208 class | High-turn winding (several ohms) keeps currents low on a bench supply. Same batch. |
| Bench DC supply, 0–30 V, ≥3 A, with current limit | Constant-current mode is ideal |
| 2x K-type thermocouples + 2x MAX31855 boards | One on stator back iron / housing per motor, one for room air |
| Arduino or ESP32 | Logs thermocouples; or read them by hand every 30 s |
| Multimeter (4-wire if possible) | Cold resistance R0, and V across the motor leads during the run |
| Thermally conductive potting epoxy, ~0.7 W/m·K or better | Check the datasheet value; record it |
| Kapton tape, a small aluminum plate | Mount both motors identically on the same plate size |

## Method
1. **Mount** both motors on identical small aluminum plates, same orientation, away from drafts.
   Lock the rotor (tape or a clamp). Wire DC through two phases (A–B); the third floats.
2. **Cold resistance:** at room temperature, measure line-to-line R0 and record the room
   temperature T0. Do this before every run.
3. **Baseline run, both motors, unpotted:** set a constant current giving roughly 5–10 W.
   Log `t_s, V, I, T_h, T_amb` every 10 s until temperatures are flat (at least 3 housing time
   constants, typically 30–60 min). Keep the winding below ~110 °C (magnets).
4. **Pot motor B only:** fill the slot openings and end turns with epoxy, degassing if you can;
   cure fully. Motor A stays unpotted as the control.
5. **Repeat step 3** on both motors at the same current.
6. **Analyze:**
   `python experiments/bench_rwh.py B_before.csv B_after.csv --R0 <R0b> <R0a> --T0 <T0b> <T0a>`
   and the same for motor A (the control).

Winding temperature comes from the resistance rise (copper, 0.393%/K), which averages over the
whole winding, so no sensor inside the motor is needed.

## What to report
- `R_wh` and `R_ha` before and after, for both motors, plus the fitted time constants.
- The control motor's drift (repeatability of the setup).
- Then feed the measured `R_wh` ratio into the simulation: `joint_params(scale={"R_wh": ratio})`
  in experiments/run_cartridge.py `joint` to see what it is worth in boxes per shift.

## Safety
Current-limit the supply; never leave a run unattended above 100 °C; cure epoxy in a ventilated
area; potting is irreversible, so use motor B only.
