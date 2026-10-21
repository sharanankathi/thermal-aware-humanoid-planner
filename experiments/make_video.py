"""Side-by-side replay video: B2 round-robin vs the proposed controller, same box stream.

Usage: python experiments/make_video.py [scenario] [seed]
Writes figures/replay_<scenario>.mp4 (about 40 s). The shift is sampled at evenly spaced
times; each sampled pick is shown in full at ~2.5x real speed, with a temperature gauge for
each arm's hottest joint. When a controller hits a protective stop its panel freezes.
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "osmesa")          # headless Linux: software GL
    os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")  # macOS uses its native GL
os.environ.setdefault("MENAGERIE_PATH", str(Path.home() / "mujoco_menagerie"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sim.controllers import RoundRobin, ThermalBudget  # noqa: E402
from sim.cycles import PICK_SLOTS, PLACE_SLOTS, SEGMENTS, CycleLibrary  # noqa: E402
from sim.robot import Robot  # noqa: E402
from sim.shift import SCENARIOS, joint_params, loss_coeff, run_shift  # noqa: E402

W, H, FPS = 480, 400, 15
N_SAMPLES = 14          # picks shown per panel
SPEED = 2.5             # playback speed within a shown pick
T_LIM, T_AMB = 140.0, 25.0


def font(sz):
    for f in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"):
        if Path(f).exists():
            return ImageFont.truetype(f, sz)
    return ImageFont.load_default()


F_S, F_M, F_L = font(13), font(16), font(22)


def cycle_frames(robot, lib, ev):
    """List of (q14, box_pos or None) for one pick at FPS/SPEED sampling."""
    t0, box, plan, _ = ev
    cyc = lib.get(box.pick, box.place, plan.arm, plan.posture, plan.pace)
    wp = cyc.waypoints
    out = []
    for (a, b, _, held), T in zip(SEGMENTS, cyc.durations):
        n = max(int(round(T / SPEED * FPS)), 1)
        for s in np.linspace(0, 1, n, endpoint=False):
            h = 10 * s**3 - 15 * s**4 + 6 * s**5
            q = wp[a] + h * (wp[b] - wp[a])
            out.append((q, held, a, b))
    return out, box, plan


def box_position(robot, q, held, a, plan, box):
    if held:
        sides = ("l", "r") if plan.arm == "both" else (plan.arm,)
        p = np.mean([robot.grasp_pos(q, s) for s in sides], axis=0)
        return p
    if a in ("home", "pick_above", "pick") and not held:
        return np.array(PICK_SLOTS[box.pick])
    return np.array(PLACE_SLOTS[box.place])


def gauge(draw, x, y, label, T):
    w, h = 100, 12
    frac = np.clip((T - T_AMB) / (T_LIM - T_AMB), 0, 1)
    col = (42, 120, 214) if T < 115 else ((237, 161, 0) if T < 132 else (227, 73, 72))
    draw.rectangle([x, y, x + w, y + h], outline=(82, 81, 78), width=1)
    draw.rectangle([x + 1, y + 1, x + 1 + int((w - 2) * frac), y + h - 1], fill=col)
    draw.text((x + w + 6, y - 2), f"{label} {T:4.0f} °C", font=F_S, fill=(11, 11, 11))


def render_panel(robot, renderer, cam, q, boxp, title, clock_min, nbox, Tw, stopped=None):
    m, d = robot.model, robot.data
    robot.set_q(q)
    vis = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "box_vis")
    d.mocap_pos[m.body_mocapid[vis]] = boxp
    mujoco.mj_forward(m, d)
    renderer.update_scene(d, camera=cam)
    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0       # 5x faster in software GL
    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
    img = Image.fromarray(renderer.render())
    canvas = Image.new("RGB", (W, H + 90), (252, 252, 251))
    canvas.paste(img, (0, 90))
    dr = ImageDraw.Draw(canvas)
    dr.text((10, 6), title, font=F_L, fill=(11, 11, 11))
    dr.text((10, 36), f"shift clock {clock_min:5.1f} min   boxes {nbox:4d}", font=F_M,
            fill=(82, 81, 78))
    gauge(dr, 10, 62, "L arm", float(Tw[:7].max()))
    gauge(dr, 245, 62, "R arm", float(Tw[7:].max()))
    if stopped is not None:
        overlay = Image.new("RGBA", canvas.size, (0, 0, 0, 0))
        od = ImageDraw.Draw(overlay)
        od.rectangle([0, H // 2 + 40, W, H // 2 + 120], fill=(227, 73, 72, 225))
        canvas = Image.alpha_composite(canvas.convert("RGBA"), overlay).convert("RGB")
        dr = ImageDraw.Draw(canvas)
        dr.text((20, H // 2 + 52), "PROTECTIVE STOP", font=F_L, fill=(255, 255, 255))
        dr.text((20, H // 2 + 86), f"winding at limit after {stopped:.0f} min", font=F_M,
                fill=(255, 255, 255))
    return canvas


def main(scn="warm_heavy", seed=0):
    robot = Robot.load()
    lib = CycleLibrary(robot, loss_coeff(joint_params()), verbose=False)
    sc = SCENARIOS[scn]
    runs = [("B2 round-robin", run_shift(lib, RoundRobin(), sc, seed)),
            ("Proposed", run_shift(lib, ThermalBudget(), sc, seed))]
    robot.model.vis.headlight.ambient[:] = [0.45, 0.45, 0.45]
    robot.model.vis.headlight.diffuse[:] = [0.6, 0.6, 0.6]
    renderer = mujoco.Renderer(robot.model, H, W)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.2, 0.0, 0.98]
    cam.distance, cam.azimuth, cam.elevation = 2.5, -65, -9
    times = np.linspace(0, sc.shift_s - 60, N_SAMPLES)

    def frame_specs(title, res):
        """Yield render arguments for every frame of one panel (nothing rendered yet)."""
        ev = res.trace_events
        starts = np.array([e[0] for e in ev])
        for tk in times:
            stopped = res.stopped and tk > res.t_stop - 10
            i = len(ev) - 1 if stopped else min(int(np.searchsorted(starts, tk)), len(ev) - 1)
            seq, box, plan = cycle_frames(robot, lib, ev[i])
            if stopped:
                q0, held, a, _ = seq[0]
                spec = (q0, box_position(robot, q0, held, a, plan, box), title, res.t_stop / 60,
                        res.boxes, res.trace_T[-1], res.t_stop / 60)
                for _ in seq:
                    yield spec
                continue
            for q, held, a, _ in seq:
                yield (q, box_position(robot, q, held, a, plan, box), title, ev[i][0] / 60, i,
                       ev[i][3], None)

    def draw(spec):
        q, bp, title, clock, nbox, Tw, stopped = spec
        return render_panel(robot, renderer, cam, q, bp, title, clock, nbox, Tw, stopped)

    out_dir = ROOT / "figures" / "frames"
    out_dir.mkdir(parents=True, exist_ok=True)
    for f in out_dir.glob("*.png"):
        f.unlink()
    gens = [frame_specs(title, res) for title, res in runs]
    last = [None, None]
    cache = [None, None]          # frozen (stopped) panels are rendered once
    n = 0
    while True:
        specs = [next(g, None) for g in gens]
        if all(sp is None for sp in specs):
            break
        img = Image.new("RGB", (2 * W + 6, H + 90), (228, 227, 223))
        for p_, sp in enumerate(specs):
            if sp is None:
                panel = last[p_]
            elif sp[6] is not None:
                if cache[p_] is None:
                    cache[p_] = draw(sp)
                panel = cache[p_]
            else:
                panel = draw(sp)
            last[p_] = panel
            img.paste(panel, (p_ * (W + 6), 0))
        img.save(out_dir / f"{n:05d}.png")
        n += 1
        if n % 100 == 0:
            print(f"  {n} frames", flush=True)
    out = ROOT / "figures" / f"replay_{scn}.mp4"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS), "-i",
                    str(out_dir / "%05d.png"), "-c:v", "libx264", "-pix_fmt", "yuv420p",
                    "-crf", "23", str(out)], check=True)
    print("wrote", out, f"({n / FPS:.0f} s)")


if __name__ == "__main__":
    scn = sys.argv[1] if len(sys.argv) > 1 else "warm_heavy"
    main(scn, int(sys.argv[2]) if len(sys.argv) > 2 else 0)
