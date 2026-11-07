#!/usr/bin/env python3
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import compare_maps as cm


def signed_dist(world, X, Y):
    d = np.full(X.shape, np.inf)
    for o in world["obstacles"]:
        if o["type"] == "cyl":
            dd = np.hypot(X - o["x"], Y - o["y"]) - o["r"]
        else:
            c, s = math.cos(-o["yaw"]), math.sin(-o["yaw"])
            dx, dy = X - o["x"], Y - o["y"]
            qx = np.abs(c * dx - s * dy) - o["sx"] / 2
            qy = np.abs(s * dx + c * dy) - o["sy"] / 2
            dd = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0)
        d = np.minimum(d, dd)
    return d


prefix = sys.argv[1]
res = json.load(open(prefix + ".json"))
world = cm.make(res["seed"], 10.0, res.get("obstacles", 12))
for name, f in [("ours", prefix + "_map.npz"), ("slam_toolbox", prefix + "_slammap.npz")]:
    if not os.path.exists(f):
        continue
    p = cm.occupied_world_points(f, world["start"])
    d = signed_dist(world, p[:, 0], p[:, 1])
    print(f"{name}: {len(d)} occupied cells, median {np.median(d) * 100:+.1f} cm")
    edges = [-1e9, -0.20, -0.10, -0.05, 0.0, 0.05, 0.10, 0.20, 0.50, 1e9]
    labels = ["< -20", "-20..-10", "-10..-5", "-5..0", "0..5", "5..10", "10..20", "20..50", "> 50"]
    counts, _ = np.histogram(d, bins=edges)
    for lab, n in zip(labels, counts):
        print(f"  {lab:>9} cm  {n:5d}  {'#' * int(60 * n / len(d))}")
