# Forward-simulation cross-check

Forward simulation at 1 kHz, PID tracking at Apollo's position gains, torque clipped to Apollo limits. 'Box known': inverse-dynamics feedforward including the box. 'Box unknown': feedforward of the arm alone, so feedback must carry the payload. Compared with the 8-node inverse-dynamics numbers used in every experiment; time to 140 °C = the cycle repeated from a cold start at 25 °C.

| Cycle | Joint | ID 8-node RMS τ (Nm) | ID dense RMS τ (Nm) | Fwd, box known RMS τ (Nm) | Fwd, box unknown RMS τ (Nm) | Heat diff., box unknown vs 8-node |
|---|---|---|---|---|---|---|
| right arm, 8 kg, high shelf | r_shoulder_fe | 44.8 | 44.8 | 44.8 | 45.4 | +2.5% |
| right arm, 8 kg, high shelf | r_wrist_roll | 3.4 | 3.4 | 3.4 | 7.0 | +319.9% |
| two-arm, 8 kg, high shelf | l_shoulder_fe | 29.3 | 29.3 | 29.3 | 30.5 | +8.8% |
| two-arm, 8 kg, high shelf | r_shoulder_fe | 29.2 | 29.2 | 29.2 | 30.3 | +7.5% |
| right arm, 2 kg, std shelf | r_shoulder_fe | 16.7 | 16.7 | 16.7 | 16.9 | +1.5% |
| right arm, 2 kg, std shelf | r_elbow_fe | 16.1 | 16.1 | 16.1 | 16.6 | +6.7% |
| right arm, 8 kg, energy-opt posture | r_shoulder_fe | 42.2 | 42.2 | 42.2 | 44.3 | +10.1% |
| right arm, 8 kg, energy-opt posture | r_wrist_roll | 3.8 | 3.8 | 3.8 | 7.8 | +318.8% |

| Cycle | Max track. err., known / unknown (deg) | Copper loss vs 8-node, known / unknown | Time to 140 °C: 8-node / fwd known / fwd unknown | Max quadrature error in RMS τ (Nm) |
|---|---|---|---|---|
| right arm, 8 kg, high shelf | 0.14 / 71.51 | -0.0% / +98.6% | 10.0 min / 10.0 min / 1.7 min | 0.000 |
| two-arm, 8 kg, high shelf | 0.14 / 19.22 | +0.0% / +23.5% | never (2 h) / never (2 h) / never (2 h) | 0.000 |
| right arm, 2 kg, std shelf | 0.10 / 9.08 | +0.0% / +6.1% | never (2 h) / never (2 h) / never (2 h) | 0.000 |
| right arm, 8 kg, energy-opt posture | 0.15 / 77.98 | +0.1% / +156.9% | 13.2 min / 13.1 min / 1.3 min | 0.000 |
