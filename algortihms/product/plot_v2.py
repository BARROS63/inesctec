import sys
import json
import csv
import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches

SHOW_INFLATED = False  # set True if you want to show safety margin as dashed outline

def read_csv_to_dictlists(path):
    out = {}
    if not os.path.exists(path):
        print(f"[WARN] Missing file: {path}")
        return out
    with open(path, newline='') as fh:
        reader = csv.DictReader(fh)
        cols = reader.fieldnames or []
        out = {k: [] for k in cols}
        for row in reader:
            for k, v in row.items():
                try:
                    out[k].append(float(v))
                except (TypeError, ValueError):
                    out[k].append(v)
    return out

def read_json(path):
    if not os.path.exists(path):
        print(f"[WARN] Missing file: {path}")
        return {}
    with open(path) as fh:
        return json.load(fh)

# usage: python3 plot.py obstacles.csv lazy.csv bspline.csv meta.json
obs_file  = sys.argv[1] if len(sys.argv) > 1 else "obstacles.csv"
lazy_file = sys.argv[2] if len(sys.argv) > 2 else "lazy.csv"
bs_file   = sys.argv[3] if len(sys.argv) > 3 else "bspline.csv"
meta_file = sys.argv[4] if len(sys.argv) > 4 else "meta.json"

obs  = read_csv_to_dictlists(obs_file)
lazy = read_csv_to_dictlists(lazy_file)
bs   = read_csv_to_dictlists(bs_file)
meta = read_json(meta_file)

cageL = meta.get("cageL", 8.0)
cageW = meta.get("cageW", 4.0)

# Basic sanity checks
for required, data in [("lazy.csv", lazy), ("bspline.csv", bs), ("obstacles.csv", obs)]:
    if not data:
        print(f"[ERROR] {required} not loaded; cannot plot properly.")
        sys.exit(1)

fig, ax = plt.subplots(figsize=(10, 5))

# cage
ax.add_patch(patches.Rectangle((0, 0), cageL, cageW, fill=False, linewidth=2))

# Obstacles: REAL size filled
cx_list = obs.get("cx", [])
cy_list = obs.get("cy", [])
half_real_list = obs.get("half_real", obs.get("half", []))  # fallback to older column name
half_inf_list  = obs.get("half_inflated", [])

for i, (cx, cy, half_real) in enumerate(zip(cx_list, cy_list, half_real_list)):
    ax.add_patch(
        patches.Rectangle(
            (cx - half_real, cy - half_real),
            2 * half_real, 2 * half_real,
            color="orange",
            alpha=0.55,
            label="Obstacle (real)" if i == 0 else None
        )
    )

# Optional inflated outline (safety buffer)
if SHOW_INFLATED and half_inf_list:
    for i, (cx, cy, half_inf) in enumerate(zip(cx_list, cy_list, half_inf_list)):
        ax.add_patch(
            patches.Rectangle(
                (cx - half_inf, cy - half_inf),
                2 * half_inf, 2 * half_inf,
                fill=False,
                linestyle="--",
                linewidth=1.5,
                alpha=0.7,
                label="Safety buffer (inflated)" if i == 0 else None
            )
        )

# Paths
ax.plot(lazy["x"], lazy["y"], label="Lazy Theta* (planned path)", linewidth=2)
ax.plot(bs["x"], bs["y"], label="B-spline (smoothed path)", linewidth=2.5)

# start/goal
ax.scatter([0, cageL], [0, cageW], s=80)
ax.text(0, 0, "A", fontsize=12)
ax.text(cageL, cageW, "B", fontsize=12)

ax.set_aspect("equal", adjustable="box")
ax.set_xlim(-0.5, cageL + 0.5)
ax.set_ylim(-0.5, cageW + 0.5)
ax.grid(True, alpha=0.3)
ax.legend()
ax.set_title(f"Lazy Theta* planning + B-spline smoothing ({cageL}m x {cageW}m cage)")

plt.savefig("path.png", dpi=220)
