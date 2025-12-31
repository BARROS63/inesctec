import sys
import json
import csv
import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches

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

# --------- Read from CLI args if given ----------
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



fig, ax = plt.subplots(figsize=(8, 4))

# cage
ax.add_patch(patches.Rectangle((0, 0), cageL, cageW, fill=False, linewidth=2))

# obstacles: support dict-of-lists format
for cx, cy, half in zip(obs.get("cx", []), obs.get("cy", []), obs.get("half", [])):
    ax.add_patch(patches.Rectangle((cx - half, cy - half), 2 * half, 2 * half,
                                   color="orange", alpha=0.5))

# paths
ax.plot(lazy["x"], lazy["y"], label="Lazy Theta*", linewidth=2)
ax.plot(bs["x"], bs["y"], label="B-spline", linewidth=2.5)

# start/goal
ax.scatter([0, cageL], [0, cageW], s=60)
ax.text(0, 0, "A", fontsize=12)
ax.text(cageL, cageW, "B", fontsize=12)

ax.set_aspect("equal", adjustable="box")
ax.set_xlim(-0.5, cageL + 0.5)
ax.set_ylim(-0.5, cageW + 0.5)
ax.grid(True, alpha=0.3)
ax.legend()
ax.set_title(f"Lazy Theta* + B-spline ({cageL}m x {cageW}m cage)")

plt.savefig("path.png", dpi=200)
print("Saved path.png")
