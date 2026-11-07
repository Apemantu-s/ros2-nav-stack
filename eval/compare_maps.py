#!/usr/bin/env python3
import glob
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "nav_sim", "scripts"))
from make_world import make

TOL = 0.10


def dist_field(world, X, Y):
    """distance to closest obstacle (0 if inside)"""
    d = np.full(X.shape, np.inf)
    for o in world["obstacles"]:
        if o["type"] == "cyl":
            dd = np.hypot(X - o["x"], Y - o["y"]) - o["r"]
        else:
            c, s = math.cos(-o["yaw"]), math.sin(-o["yaw"])
            dx, dy = X - o["x"], Y - o["y"]
            lx, ly = c * dx - s * dy, s * dx + c * dy
            qx, qy = np.abs(lx) - o["sx"] / 2, np.abs(ly) - o["sy"] / 2
            dd = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0)
        d = np.minimum(d, dd)
    return np.maximum(d, 0)


def edge_points(world, step=0.05):
    pts = []
    for o in world["obstacles"]:
        if o["type"] == "cyl":
            n = max(8, int(2 * math.pi * o["r"] / step))
            a = np.linspace(0, 2 * math.pi, n, endpoint=False)
            pts.append(np.c_[o["x"] + o["r"] * np.cos(a), o["y"] + o["r"] * np.sin(a)])
        else:
            c, s = math.cos(o["yaw"]), math.sin(o["yaw"])
            hx, hy = o["sx"] / 2, o["sy"] / 2
            corners = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy), (hx, hy)]
            for (ax, ay), (bx, by) in zip(corners, corners[1:]):
                n = max(2, int(math.hypot(bx - ax, by - ay) / step))
                t = np.linspace(0, 1, n, endpoint=False)
                lx, ly = ax + t * (bx - ax), ay + t * (by - ay)
                pts.append(np.c_[o["x"] + c * lx - s * ly, o["y"] + s * lx + c * ly])
    return np.vstack(pts)


def occupied_world_points(npz, start):
    d = np.load(npz)
    g, r = d["grid"], float(d["resolution"])
    ox, oy = d["origin"][:2]
    ys, xs = np.nonzero(g >= 65)
    mx, my = ox + (xs + 0.5) * r, oy + (ys + 0.5) * r
    c, s = math.cos(start["yaw"]), math.sin(start["yaw"])
    return np.c_[start["x"] + c * mx - s * my, start["y"] + s * mx + c * my]


def nearest_dist(a, b):
    if len(b) == 0:
        return np.full(len(a), np.inf)
    out = np.empty(len(a))
    for i, (x, y) in enumerate(a):
        out[i] = np.min(np.hypot(b[:, 0] - x, b[:, 1] - y))
    return out


def score(res_json):
    prefix = res_json[:-5]
    res = json.load(open(res_json))
    world = make(res["seed"], 10.0, res.get("obstacles", 12))
    start = world["start"]
    # robot path in world frame
    t = np.genfromtxt(prefix + "_traj.csv", delimiter=",", names=True)
    c, s = math.cos(start["yaw"]), math.sin(start["yaw"])
    path = np.c_[start["x"] + c * t["gt_x"] - s * t["gt_y"], start["y"] + s * t["gt_x"] + c * t["gt_y"]]
    edges = edge_points(world)
    near = nearest_dist(edges, path[::5]) < 4.0
    edges = edges[near]
    out = {"seed": res["seed"], "edge_points": int(len(edges))}
    for name, f in [("ours", prefix + "_map.npz"), ("slam_toolbox", prefix + "_slammap.npz")]:
        if not os.path.exists(f):
            continue
        occ = occupied_world_points(f, start)
        prec = float((dist_field(world, occ[:, 0], occ[:, 1]) <= TOL).mean()) if len(occ) else 0.0
        cov = float((nearest_dist(edges, occ) <= TOL).mean()) if len(edges) else 0.0
        out[name] = {"occupied_cells": int(len(occ)), "precision": prec, "coverage": cov}
    return out


if __name__ == "__main__":
    files = sorted(sum((glob.glob(p) for p in sys.argv[1:]), []))
    rows = [score(f) for f in files]
    for r in rows:
        print(json.dumps(r))
    if rows:
        summary = {}
        for name in ["ours", "slam_toolbox"]:
            vals = [r[name] for r in rows if name in r]
            if vals:
                summary[name] = {k: float(np.mean([v[k] for v in vals])) for k in ["precision", "coverage"]}
                summary[name]["runs"] = len(vals)
        print("SUMMARY", json.dumps(summary))