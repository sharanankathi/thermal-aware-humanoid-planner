# Running on your Mac

```bash
cd thermal-planner
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie ~/mujoco_menagerie

pytest -q                                   # 11 tests, ~10 s
python scripts/rated_point_check.py         # thermal model vs datasheet
python experiments/run_sweep.py main        # ~10-20 min on 8-10 cores
python experiments/run_sweep.py paramerr    # ~10 min
python experiments/run_sweep.py ablation    # ~15 min
python experiments/make_figures.py          # figures/ + results/table_*.md
python experiments/make_video.py            # figures/replay_warm_heavy.mp4 (needs: brew install ffmpeg)
```

The first run builds the IK/cycle cache in `.cache/` (~1 min); later runs reuse it.
If the Menagerie folder is elsewhere: `export MENAGERIE_PATH=/path/to/mujoco_menagerie`.
Requires Python 3.10+.
