"""Cross-check: inverse-dynamics cycle torques vs closed-loop forward simulation.

Every result in the paper uses joint torques from mj_inverse along the scripted min-jerk
trajectories, sampled at 8 Gauss-Legendre nodes per segment. This script checks both parts:

  1. Quadrature: 8-node mean tau^2 vs dense (1 kHz) inverse dynamics on the same trajectory.
  2. Physics: forward simulation (mj_step, 1 kHz) of the arm tracking the same trajectory with
     computed-torque control (inverse-dynamics feedforward + PD feedback at Apollo's position
     gains), torques clipped to Apollo's limits. The torque the motors actually produce is
     compared with the inverse-dynamics prediction.
  3. Consequence: each torque set drives the thermal model through a 2-hour shift of that
     cycle; we compare the hottest winding's peak temperature.

Run: python experiments/forward_check.py   ->  results/forward_check.md
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("MENAGERIE_PATH", str(Path.home() / "mujoco_menagerie"))
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sim.cycles import SEGMENTS, CycleLibrary  # noqa: E402
from sim.robot import APOLLO_XML, JOINT_NAMES, Robot, build_spec  # noqa: E402
from sim.shift import joint_params, loss_coeff  # noqa: E402
from thermal.joint_model import JointThermalModel  # noqa: E402

DT = 0.001
# Apollo's position gains from the Menagerie XML (arm actuators, per side)
KP = np.array([395, 436, 214, 200, 47, 20, 18] * 2, float)
KD = 2.0 * np.sqrt(KP * 0.05)        # ~critically damped for 0.05 kg m^2 effective inertia
KI = 4.0 * KP                        # integral action (box-unknown case relies on it)
CASES = [  # (label, pick slot, place slot, arm, posture, box kg)
    ("right arm, 8 kg, high shelf", 2, 4, "r", 0, 8.0),
    ("two-arm, 8 kg, high shelf", 2, 4, "both", 0, 8.0),
    ("right arm, 2 kg, std shelf", 2, 1, "r", 0, 2.0),
    ("right arm, 8 kg, energy-opt posture", 0, 3, "r", 1, 8.0),
]


def trajectory(cyc):
    """Dense min-jerk reference at DT: (q, qd, qdd, held flag) over one cycle."""
    out = []
    for (a, b, _, held), T in zip(SEGMENTS, cyc.durations):
        qa, qb = cyc.waypoints[a], cyc.waypoints[b]
        n = int(round(T / DT))
        for k in range(n):
            s = (k + 0.5) / n
            h = 10 * s**3 - 15 * s**4 + 6 * s**5
            hd = (30 * s**2 - 60 * s**3 + 30 * s**4) / T
            hdd = (60 * s - 180 * s**2 + 120 * s**3) / T**2
            out.append((qa + h * (qb - qa), hd * (qb - qa), hdd * (qb - qa), held))
    return out


def arm_only_model(robot):
    """Same robot with every non-arm joint welded and payload bodies kept."""
    spec = build_spec(APOLLO_XML)
    for a in list(spec.actuators):
        spec.delete(a)
    for j in list(spec.joints):
        if j.name not in JOINT_NAMES:
            spec.delete(j)
    m = spec.compile()
    m.opt.timestep = DT
    m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONSTRAINT
    return m


def main():
    robot = Robot.load()
    params = joint_params()
    lib = CycleLibrary(robot, loss_coeff(params), verbose=False)
    m = arm_only_model(robot)
    d = mujoco.MjData(m)
    jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINT_NAMES]
    qadr = np.array([m.jnt_qposadr[j] for j in jid])
    dadr = np.array([m.jnt_dofadr[j] for j in jid])
    pay = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{s}_payload") for s in "lr"}
    tau_max = robot.tau_max
    # stand-pose rest values for the welded joints are baked in by build_spec's qpos0? No:
    # welding removes those DOF at their *default* (zero) angle, so compare ID on this same
    # model rather than on Robot (legs/torso positions do not affect arm torques on a fixed
    # pelvis, but torso joints would: confirm they are zero at stand).
    lines = ["# Forward-simulation cross-check\n",
             "Forward simulation at 1 kHz, PID tracking at Apollo's position gains, torque clipped to "
             "Apollo limits. 'Box known': inverse-dynamics feedforward including the box. "
             "'Box unknown': feedforward of the arm alone, so feedback must carry the payload. "
             "Compared with the 8-node inverse-dynamics numbers used in every experiment; "
             "time to 140 °C = the cycle repeated from a cold start at 25 °C.\n",
             "| Cycle | Joint | ID 8-node RMS τ (Nm) | ID dense RMS τ (Nm) | Fwd, box known RMS τ (Nm) "
             "| Fwd, box unknown RMS τ (Nm) | Heat diff., box unknown vs 8-node |",
             "|---|---|---|---|---|---|---|"]
    summary = []
    for label, i, j, arm, k, mass in CASES:
        cyc = lib.get(i, j, arm, k, 1.0)
        sides = ("l", "r") if arm == "both" else (arm,)
        ref = trajectory(cyc)

        def set_payload(held):
            for s in "lr":
                mm = mass / len(sides) if (held and s in sides) else 1e-6
                I = mm * 0.24**2 / 6
                m.body_mass[pay[s]] = mm
                m.body_inertia[pay[s]] = [I, I, I]

        # dense inverse dynamics on this model
        tau_id = np.zeros((len(ref), 14))
        for n, (q, qd, qdd, held) in enumerate(ref):
            set_payload(held)
            d.qpos[qadr], d.qvel[dadr], d.qacc[dadr] = q, qd, qdd
            mujoco.mj_inverse(m, d)
            tau_id[n] = d.qfrc_inverse[dadr]
        # feedforward that does NOT know the box (arm dynamics only)
        tau_id0 = np.zeros_like(tau_id)
        set_payload(False)
        for n, (q, qd, qdd, held) in enumerate(ref):
            d.qpos[qadr], d.qvel[dadr], d.qacc[dadr] = q, qd, qdd
            mujoco.mj_inverse(m, d)
            tau_id0[n] = d.qfrc_inverse[dadr]

        def forward(ff):
            mujoco.mj_resetData(m, d)
            d.qpos[qadr] = ref[0][0]
            d.qvel[dadr] = ref[0][1]
            out = np.zeros_like(tau_id)
            e = np.zeros(len(ref))
            ie = np.zeros(14)
            for n, (q, qd, qdd, held) in enumerate(ref):
                set_payload(held)
                ep = q - d.qpos[qadr]
                ie += ep * DT
                tau = ff[n] + KP * ep + KD * (qd - d.qvel[dadr]) + KI * ie
                tau = np.clip(tau, -tau_max, tau_max)
                d.qfrc_applied[:] = 0
                d.qfrc_applied[dadr] = tau
                out[n] = tau
                mujoco.mj_step(m, d)
                e[n] = np.abs(d.qpos[qadr] - q).max()
            return out, e

        tau_fw, err = forward(tau_id)
        tau_fw0, err0 = forward(tau_id0)
        # cycle mean tau^2 three ways
        w = np.full(len(ref), DT)
        ms_dense = (tau_id**2 * w[:, None]).sum(0) / w.sum()
        ms_fw = (tau_fw**2 * w[:, None]).sum(0) / w.sum()
        ms_fw0 = (tau_fw0**2 * w[:, None]).sum(0) / w.sum()
        ms_gl = cyc.mean_tau_sq(mass)
        if arm != "both":   # idle arm: the experiments hold it at the stand pose
            other = slice(7, 14) if arm == "l" else slice(0, 7)
            idle = lib.idle_tau_sq()
            for ms in (ms_dense, ms_fw, ms_fw0, ms_gl):
                ms[other] = idle[other]
        # thermal consequence: time for the hottest winding to reach 140 C, cycle repeated
        th = JointThermalModel(params, T_amb=25.0)
        ttl = [th.forecast_ttl(ms[None, :], horizon=7200, dt=2.0)[0].min() / 60
               for ms in (ms_gl, ms_fw, ms_fw0)]
        c = loss_coeff(params)
        heat_gl, heat_fw, heat_fw0 = c * ms_gl, c * ms_fw, c * ms_fw0
        idx = sorted(range(14), key=lambda q: -heat_gl[q])[:2]
        for q in idx:
            lines.append(f"| {label} | {JOINT_NAMES[q]} | {np.sqrt(ms_gl[q]):.1f} | "
                         f"{np.sqrt(ms_dense[q]):.1f} | {np.sqrt(ms_fw[q]):.1f} | "
                         f"{np.sqrt(ms_fw0[q]):.1f} | {heat_fw0[q] / heat_gl[q] - 1:+.1%} |")
        fmt = lambda x: "never (2 h)" if np.isinf(x) else f"{x:.1f} min"
        summary.append((label, err.max() * 180 / np.pi, err0.max() * 180 / np.pi,
                        heat_fw.sum() / heat_gl.sum() - 1, heat_fw0.sum() / heat_gl.sum() - 1,
                        fmt(ttl[0]), fmt(ttl[1]), fmt(ttl[2]),
                        float(np.abs(np.sqrt(ms_dense) - np.sqrt(ms_gl)).max())))
    lines += ["", "| Cycle | Max track. err., known / unknown (deg) | Copper loss vs 8-node, known / unknown | "
              "Time to 140 °C: 8-node / fwd known / fwd unknown | Max quadrature error in RMS τ (Nm) |",
              "|---|---|---|---|---|"]
    for label, e, e0, dh, dh0, t0, t1, t2, qe in summary:
        lines.append(f"| {label} | {e:.2f} / {e0:.2f} | {dh:+.1%} / {dh0:+.1%} | {t0} / {t1} / {t2} | {qe:.3f} |")
    out = ROOT / "results" / "forward_check.md"
    out.write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
