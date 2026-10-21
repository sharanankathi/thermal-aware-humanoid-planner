"""Portfolio hero render (1920x1080): Apollo mid two-arm lift at the high shelf.

Usage: python experiments/make_hero.py  ->  figures/hero.png
Scene dressing (light backdrop, shadows, stacked boxes, conveyor) is added here only;
the simulation model is unchanged.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "osmesa")
    os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")
os.environ.setdefault("MENAGERIE_PATH", str(Path.home() / "mujoco_menagerie"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sim.cycles import PLACE_SLOTS, CycleLibrary  # noqa: E402
from sim.robot import APOLLO_XML, FLOOR_Z, JOINT_NAMES, Robot, build_spec  # noqa: E402
from sim.shift import joint_params, loss_coeff  # noqa: E402

W, H = 1920, 1080
CARD = [0.80, 0.62, 0.40, 1]
WOOD = [0.62, 0.52, 0.42, 1]


def dressed_spec():
    spec = build_spec(APOLLO_XML)
    spec.visual.global_.offwidth, spec.visual.global_.offheight = W, H
    spec.visual.quality.shadowsize = 8192
    spec.visual.headlight.ambient = [0.45, 0.45, 0.47]
    spec.visual.headlight.diffuse = [0.35, 0.35, 0.35]
    spec.visual.headlight.specular = [0.05, 0.05, 0.05]
    spec.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                     rgb1=[0.97, 0.97, 0.96], rgb2=[0.86, 0.87, 0.89], width=512, height=512)
    spec.add_texture(name="floortex", type=mujoco.mjtTexture.mjTEXTURE_2D,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, rgb1=[0.93, 0.93, 0.92],
                     rgb2=[0.88, 0.88, 0.87], width=512, height=512)
    spec.add_material(name="floormat", textures=["", "floortex"], texrepeat=[6, 6],
                      reflectance=0.0)
    wb = spec.worldbody
    for lt in list(spec.lights):
        lt.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
        lt.diffuse = [0.2, 0.2, 0.2]
        lt.castshadow = False
    for g in list(spec.geoms):
        if g.name in ("shelf_std", "shelf_high"):
            g.rgba = WOOD
        if g.name == "pick_table":        # same 0.85 m top, drawn as slab + legs
            g.rgba = [0, 0, 0, 0]
        if g.name == "floor":
            g.rgba = [1, 1, 1, 1]
            g.material = "floormat"
            g.size = [8, 8, 0.05]
    wb.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, pos=[0.36, 0, 0.83], size=[0.14, 0.40, 0.02],
                rgba=WOOD, contype=0, conaffinity=0)
    for x in (0.25, 0.47):
        for y in (-0.37, 0.37):
            wb.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, pos=[x, y, (0.81 + FLOOR_Z) / 2],
                        size=[0.02, 0.02, (0.81 - FLOOR_Z) / 2], rgba=[0.35, 0.35, 0.37, 1], contype=0,
                        conaffinity=0)
    # shelf uprights
    for y in (-0.47, 0.47):
        wb.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, pos=[0.50, y, (1.50 + FLOOR_Z) / 2],
                    size=[0.03, 0.02, (1.50 - FLOOR_Z) / 2],
                    rgba=[0.35, 0.35, 0.37, 1], contype=0, conaffinity=0)
    # boxes already placed on the high and standard shelves
    for (x, y, z) in [(0.46, 0.32, 1.54), (0.48, -0.30, 1.16), (0.48, 0.30, 1.16)]:
        wb.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, pos=[x, y, z], size=[0.10, 0.11, 0.12],
                    rgba=CARD, contype=0, conaffinity=0)
    # boxes waiting on the pick table
    for y in (-0.27,):
        wb.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, pos=[0.36, y, 0.97], size=[0.10, 0.11, 0.12],
                    rgba=CARD, contype=0, conaffinity=0)
    key = wb.add_light(pos=[1.0, 2.0, 4.0], dir=[-0.2, -0.45, -0.87], diffuse=[0.55, 0.54, 0.52],
                       specular=[0.05, 0.05, 0.05], castshadow=True)
    key.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    fill = wb.add_light(pos=[-1.0, -2.0, 2.5], dir=[0.3, 0.6, -0.7], diffuse=[0.22, 0.23, 0.26],
                        castshadow=False)
    fill.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    return spec


def main(az=-58.0, el=-8.0, dist=2.55, lookat=(0.22, 0.0, 1.05), out_name="hero.png",
         scale=1.0):
    base = Robot.load()
    lib = CycleLibrary(base, loss_coeff(joint_params()), verbose=False)
    m = dressed_spec().compile()
    m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONSTRAINT
    d = mujoco.MjData(m)
    # pose: two-arm grasp, partway between "above place" and "place" on the high shelf
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
    mujoco.mj_forward(m, d)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = lookat
    cam.distance, cam.azimuth, cam.elevation = dist, az, el
    opt = mujoco.MjvOption()
    opt.sitegroup[:] = 0                 # hide grasp-point markers
    r = mujoco.Renderer(m, int(H * scale), int(W * scale))
    r.update_scene(d, camera=cam, scene_option=opt)
    r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = True
    r.scene.flags[mujoco.mjtRndFlag.mjRND_SKYBOX] = True
    img = r.render()
    out = ROOT / "figures" / out_name
    Image.fromarray(img).save(out)
    print("wrote", out)
    return out


if __name__ == "__main__":
    main()
