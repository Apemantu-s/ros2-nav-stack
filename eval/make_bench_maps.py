#!/usr/bin/env python3
"""make pgm maps of the real worlds for the planner benchmark

usage: python3 eval/make_bench_maps.py --seeds 0-199 --out results/bench_maps
also writes cases.txt for bench_planners
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "nav_sim", "scripts"))
from make_world import make

ap = argparse.ArgumentParser()
ap.add_argument("--seeds", default="0-199")
ap.add_argument("--obstacles", type=int, default=12)
ap.add_argument("--res", type=float, default=0.05)
ap.add_argument("--out", default="results/bench_maps")
a = ap.parse_args()
lo, hi = map(int, a.seeds.split("-"))
os.makedirs(a.out, exist_ok=True)

size = 11.0
n = int(size / a.res)
origin = -size / 2
lines = []
for seed in range(lo, hi + 1):
    w = make(seed, 10.0, a.obstacles)
    occ = np.zeros((n, n), dtype=bool)
    xs = origin + (np.arange(n) + 0.5) * a.res
    X, Y = np.meshgrid(xs, xs)
    for o in w["obstacles"]:
        if o["type"] == "cyl":
            occ |= np.hypot(X - o["x"], Y - o["y"]) <= o["r"]
        else:
            c, s = np.cos(-o["yaw"]), np.sin(-o["yaw"])
            dx, dy = X - o["x"], Y - o["y"]
            lx, ly = c * dx - s * dy, s * dx + c * dy
            occ |= (np.abs(lx) <= o["sx"] / 2) & (np.abs(ly) <= o["sy"] / 2)
    img = np.where(occ, 0, 255).astype(np.uint8)[::-1]  # PGM row 0 = top
    path = os.path.join(a.out, f"world_{seed:04d}.pgm")
    with open(path, "wb") as f:
        f.write(b"P5 %d %d 255\n" % (n, n) + img.tobytes())
    st, g = w["start"], w["goal"]
    lines.append(f"{path} {a.res} {st['x'] - origin} {st['y'] - origin} "
                 f"{g['x'] - origin} {g['y'] - origin}")
with open(os.path.join(a.out, "cases.txt"), "w") as f:
    f.write("\n".join(lines) + "\n")
print("wrote", len(lines), "maps")