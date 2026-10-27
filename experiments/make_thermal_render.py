"""Portfolio render: Apollo with each arm joint coloured by its peak winding temperature over a
2-hour heavy-box shift in a 35 C room. Left: starting gearing. Right: re-geared joints. Both air-cooled,
both run by the same 130 C reactive limb switch, so the re-geared robot spends its headroom on throughput.

Usage: python experiments/make_thermal_render.py  ->  figures/report_thermal_robot.png
Needs an OpenGL context (works on macOS; on headless Linux set MUJOCO_GL=osmesa or egl).
Scene dressing and pose come from experiments/make_hero.py; the simulation model is unchanged.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))
os.environ.setdefault("MENAGERIE_PATH", str(Path.home() / "mujoco_menagerie"))

import make_hero  # noqa: E402  (sets MUJOCO_GL on headless Linux)
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from sim.controllers import ReactiveModeSwitch  # noqa: E402
from sim.cycles import CycleLibrary  # noqa: E402
from sim.gearing import OMEGA_MAX, _kinematics, regear_library  # noqa: E402
from sim.robot import JOINT_NAMES, Robot  # noqa: E402
from sim.shift import SCENARIOS, joint_params, loss_coeff, run_shift  # noqa: E402

T_LO, T_HI = 30.0, 140.0          # colour scale (C); 140 C = protective stop
CMAP = plt.get_cmap("coolwarm")
W, H = 1100, 1300


def peak_temps():
    r = Robot.load()
    P = joint_params()
    lib = CycleLibrary(r, loss_coeff(P), verbose=False)
    N0 = np.array([p.N for p in P])
    qmax = np.zeros(14)
    for k, c in lib.cycles.items():
        if k[4] == 1.0:
            qmax = np.maximum(qmax, np.abs(_kinematics(c)[0]).max(axis=(0, 1)))
    mult = np.minimum(np.floor(OMEGA_MAX / (N0 * qmax) * 0.98 * 100) / 100, 2.0)
    base = regear_library(lib, r, P, np.ones(14), "fixed")[0]
    geared = regear_library(lib, r, P, mult, "fixed")[0]
    sc = SCENARIOS["warm_heavy"]
    out = {}
    for name, L in (("base", base), ("geared", geared)):
        res = run_shift(L, ReactiveModeSwitch(130.0, 120.0, 135.0), sc, 0, record=False)
        out[name] = (res.peak_T.copy(), res.boxes)
    return out, lib


def render(temps, lib):
    base = Robot.load()
    m = make_hero.dressed_spec().compile()
    m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONSTRAINT
    d = mujoco.MjData(m)
    cyc = lib.get(2, 4, "both", 0, 1.0)
    q = 0.6 * cyc.waypoints["place_above"] + 0.4 * cyc.waypoints["place"]
    d.qpos[:] = base.qpos0
    for k, name in enumerate(JOINT_NAMES):
        d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)]] = q[k]
    mujoco.mj_forward(m, d)
    gl = d.site_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "l_grasp")]
    gr = d.site_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "r_grasp")]
    vis = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "box_vis")
    d.mocap_pos[m.body_mocapid[vis]] = (gl + gr) / 2
    # tint every geom of the body each arm joint drives by that joint's peak winding temperature
    for k, name in enumerate(JOINT_NAMES):
        body = m.jnt_bodyid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)]
        rgba = np.array(CMAP(np.clip((temps[k] - T_LO) / (T_HI - T_LO), 0, 1)))
        for g in np.where(m.geom_bodyid == body)[0]:
            if m.geom_rgba[g, 3] == 0:          # invisible collision geoms stay invisible
                continue
            m.geom_matid[g] = -1
            m.geom_rgba[g] = rgba
    mujoco.mj_forward(m, d)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (0.10, 0.0, 1.15)
    cam.distance, cam.azimuth, cam.elevation = 2.1, -25.0, -22.0     # behind and above the robot, shelf beyond
    opt = mujoco.MjvOption()
    opt.sitegroup[:] = 0
    m.vis.global_.offwidth, m.vis.global_.offheight = max(W, 1920), max(H, 1080)
    r = mujoco.Renderer(m, H, W)
    r.update_scene(d, camera=cam, scene_option=opt)
    r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = True
    r.scene.flags[mujoco.mjtRndFlag.mjRND_SKYBOX] = True
    return r.render()


def main():
    data, lib = peak_temps()
    imgs = {k: render(t, lib) for k, (t, _) in data.items()}
    fig = plt.figure(figsize=(11, 7.4), facecolor="#fcfcfb")
    titles = {"base": "Starting gearing", "geared": "Re-geared joints"}
    for i, k in enumerate(("base", "geared")):
        ax = fig.add_axes([0.02 + i * 0.47, 0.10, 0.45, 0.78])
        ax.imshow(imgs[k])
        ax.axis("off")
        t, boxes = data[k]
        ax.set_title(f"{titles[k]}\nhottest joint {t.max():.0f} °C  ·  {boxes:,} boxes in 2 h",
                     fontsize=12, color="#111418", loc="left")
    cax = fig.add_axes([0.25, 0.06, 0.5, 0.025])
    sm = plt.cm.ScalarMappable(cmap=CMAP, norm=plt.Normalize(T_LO, T_HI))
    cb = fig.colorbar(sm, cax=cax, orientation="horizontal")
    cb.set_label("Peak winding temperature over the shift (°C); 140 °C = protective stop",
                 fontsize=10, color="#3f4752")
    gain = data["geared"][1] / data["base"][1] - 1
    fig.suptitle(f"Same robot, same air cooling, same temperature limit: re-geared joints move {gain:.0%} more boxes",
                 x=0.02, y=0.975, ha="left", fontsize=14, fontweight="bold", color="#111418")
    out = ROOT / "figures" / "report_thermal_robot.png"
    fig.savefig(out, dpi=170)
    print(f"wrote {out}")
    for k, (t, b) in data.items():
        print(k, b, "boxes; peak per joint (left arm):", np.round(t[:7], 0))


if __name__ == "__main__":
    main()
