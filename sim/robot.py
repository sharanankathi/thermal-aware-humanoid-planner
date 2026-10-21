"""Apptronik Apollo upper body (MuJoCo Menagerie) on a fixed base, with a work cell.

- Floating base removed: pelvis welded to the world, legs/torso/neck held at the 'stand' pose.
- 14 arm joints are the only moving DOF; joint torques come from MuJoCo inverse dynamics
  (mj_inverse) along the commanded trajectory, including the held payload.
- Each hand carries a 'payload' body whose mass/inertia is set at runtime when a box is held.
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

MENAGERIE = Path(os.environ.get("MENAGERIE_PATH", Path.home() / "mujoco_menagerie"))
APOLLO_XML = MENAGERIE / "apptronik_apollo" / "apptronik_apollo.xml"

ARM_SUFFIXES = ["shoulder_aa", "shoulder_ie", "shoulder_fe", "elbow_fe",
                "wrist_roll", "wrist_yaw", "wrist_pitch"]
SIDES = ("l", "r")
JOINT_NAMES = [f"{s}_{j}" for s in SIDES for j in ARM_SUFFIXES]   # 14, left first

# grasp point in each wrist_pitch_link frame: hand COM pushed 4 cm toward the fingers
GRASP_LOCAL = {"l": (0.007, -0.038, -0.107), "r": (0.018, 0.039, -0.109)}

# work cell (world frame, robot faces +x, pelvis at origin, shoulders at z ~1.37 m)
CELL = {
    "pick_table": dict(pos=(0.36, 0.0, 0.457), size=(0.12, 0.40, 0.393)), # top at z = 0.85
    "shelf_std":  dict(pos=(0.48, 0.0, 1.02), size=(0.10, 0.45, 0.02)),   # top at z = 1.04
    "shelf_high": dict(pos=(0.46, 0.0, 1.40), size=(0.10, 0.45, 0.02)),   # top at z = 1.42
}


FLOOR_Z = 0.064   # bottom of the sole collision boxes at the stand pose (world frame)


def build_spec(xml: Path = APOLLO_XML) -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(xml))
    spec.delete(spec.joint("floating_base"))
    spec.keys[0].delete() if hasattr(spec.keys[0], "delete") else spec.delete(spec.keys[0])
    for s in SIDES:
        hand = spec.body(f"{s}_wrist_pitch_link")
        hand.add_site(name=f"{s}_grasp", pos=GRASP_LOCAL[s], size=[0.015, 0, 0],
                      rgba=[1, 0.6, 0, 1])
        pl = hand.add_body(name=f"{s}_payload", pos=GRASP_LOCAL[s])
        pl.mass = 1e-6
        pl.inertia = [1e-9, 1e-9, 1e-9]
        pl.explicitinertial = True
        pl.add_geom(name=f"{s}_payload_geom", type=mujoco.mjtGeom.mjGEOM_BOX,
                    size=[0.08, 0.08, 0.08], rgba=[0.8, 0.6, 0.3, 0.0],
                    contype=0, conaffinity=0, mass=0)
    wb = spec.worldbody
    vis = wb.add_body(name="box_vis", mocap=True, pos=[0.36, 0, 0.97])   # visual-only box
    vis.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.10, 0.12, 0.12],
                 rgba=[0.78, 0.58, 0.32, 1], contype=0, conaffinity=0, mass=0)
    for name, g in CELL.items():
        wb.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, pos=g["pos"], size=g["size"],
                    rgba=[0.55, 0.5, 0.45, 1], contype=0, conaffinity=0)
    # floor at the sole height of the stand pose (pelvis sits ~1 m above the world origin)
    wb.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.05],
                pos=[0, 0, FLOOR_Z], rgba=[0.2, 0.25, 0.3, 1], contype=0, conaffinity=0)
    wb.add_light(pos=[1, 1, 3], dir=[-0.3, -0.3, -1], diffuse=[0.8, 0.8, 0.8])
    return spec


# Stand pose (Menagerie 'stand' keyframe; joint order torso, neck, l arm, r arm, l leg, r leg)
STAND = {
    "l_shoulder_aa": 0.2, "l_shoulder_fe": 0.1, "l_elbow_fe": -0.4,
    "r_shoulder_aa": -0.2, "r_shoulder_fe": 0.1, "r_elbow_fe": -0.4,
    "l_hip_ie": 0.08, "l_hip_aa": 0.1, "l_hip_fe": -0.477, "l_knee_fe": 1.033,
    "l_ankle_ie": -0.03, "l_ankle_pd": -0.58,
    "r_hip_ie": -0.08, "r_hip_aa": -0.1, "r_hip_fe": -0.477, "r_knee_fe": 1.033,
    "r_ankle_ie": 0.03, "r_ankle_pd": -0.58,
}


@dataclass
class Robot:
    model: mujoco.MjModel
    data: mujoco.MjData
    qadr: np.ndarray          # qpos address of the 14 arm joints
    dadr: np.ndarray          # dof address of the 14 arm joints
    site: dict                # side -> grasp site id
    payload: dict             # side -> payload body id
    qpos0: np.ndarray         # full qpos with legs/torso/neck at stand
    lo: np.ndarray            # arm joint lower limits
    hi: np.ndarray            # arm joint upper limits
    tau_max: np.ndarray       # Apollo actuator force limits for the arm joints

    @classmethod
    def load(cls, xml: Path = APOLLO_XML) -> "Robot":
        m = build_spec(xml).compile()
        d = mujoco.MjData(m)
        jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINT_NAMES]
        qadr = np.array([m.jnt_qposadr[j] for j in jid])
        dadr = np.array([m.jnt_dofadr[j] for j in jid])
        qpos0 = m.qpos0.copy()
        for n, v in STAND.items():
            qpos0[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]] = v
        # free-space arm motion: no contacts or limit forces in inverse dynamics
        m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONSTRAINT
        site = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"{s}_grasp") for s in SIDES}
        payload = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{s}_payload") for s in SIDES}
        rng = np.array([m.jnt_range[j] for j in jid])
        tau_max = np.array([m.jnt_actfrcrange[j][1] for j in jid])
        d.qpos[:] = qpos0
        mujoco.mj_forward(m, d)
        return cls(m, d, qadr, dadr, site, payload, qpos0, rng[:, 0], rng[:, 1], tau_max)

    # ------------------------------------------------------------- helpers
    def arm_slice(self, side: str) -> slice:
        return slice(0, 7) if side == "l" else slice(7, 14)

    def set_q(self, q14: np.ndarray):
        self.data.qpos[:] = self.qpos0
        self.data.qpos[self.qadr] = q14

    def home(self) -> np.ndarray:
        return self.qpos0[self.qadr].copy()

    def grasp_pos(self, q14: np.ndarray, side: str) -> np.ndarray:
        self.set_q(q14)
        mujoco.mj_kinematics(self.model, self.data)
        return self.data.site_xpos[self.site[side]].copy()

    def set_payload(self, side: str, mass: float, half: float = 0.12):
        """Treat the held box as a uniform cube of half-size `half` at the grasp point."""
        b = self.payload[side]
        mass = max(mass, 1e-6)
        I = mass * (2 * half) ** 2 / 6.0
        self.model.body_mass[b] = mass
        self.model.body_inertia[b] = [I, I, I]

    def inverse_dynamics(self, q, qd, qdd) -> np.ndarray:
        """Joint torques (14,) for arm state; all other DOF static at the stand pose."""
        m, d = self.model, self.data
        self.set_q(q)
        d.qvel[:] = 0.0
        d.qacc[:] = 0.0
        d.qvel[self.dadr] = qd
        d.qacc[self.dadr] = qdd
        mujoco.mj_inverse(m, d)
        return d.qfrc_inverse[self.dadr].copy()
