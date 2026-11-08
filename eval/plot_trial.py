#!/usr/bin/env python3
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Polygon

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "nav_sim", "scripts"))
from make_world import make, box_corners  # noqa: E402


def main(prefix, out):
    res = json.load(open(prefix + ".json"))
    world = make(res["seed"], 10.0, res.get("obstacles", 12))
    s = world["start"]
    c, si = math.cos(s["yaw"]), math.sin(s["yaw"])

    def to_world(x, y):
        return s["x"] + c * x - si * y, s["y"] + si * x + c * y

    fig, ax = plt.subplots(figsize=(6, 6))
    if os.path.exists(prefix + "_map.npz"):
        d = np.load(prefix + "_map.npz")
        g = d["grid"]
        r = float(d["resolution"])
        ox, oy = d["origin"][:2]
        ys, xs = np.nonzero(g >= 65)
        wx, wy = to_world(ox + (xs + 0.5) * r, oy + (ys + 0.5) * r)
        ax.scatter(wx, wy, s=1, c="k", label="our map (occupied)")
    for o in world["obstacles"]:
        if o["type"] == "box":
            ax.add_patch(Polygon(box_corners(o), closed=True, fill=False, ec="tab:red", lw=1))
        else:
            ax.add_patch(Circle((o["x"], o["y"]), o["r"], fill=False, ec="tab:red", lw=1))
    if os.path.exists(prefix + "_traj.csv"):
        t = np.genfromtxt(prefix + "_traj.csv", delimiter=",", names=True)
        for col, name, sty in [("gt", "ground truth", "g-"), ("ekf", "EKF", "b--"), ("wheel", "wheel odom", "m:")]:
            x, y = to_world(t[col + "_x"], t[col + "_y"])
            ax.plot(x, y, sty, lw=1.5, label=name)
        if res.get("min_clearance_time") is not None and len(t) > 1:
            cx, cy = to_world(np.interp(res["min_clearance_time"], t["t"], t["gt_x"]),
                              np.interp(res["min_clearance_time"], t["t"], t["gt_y"]))
            ax.add_patch(Circle((cx, cy), 0.2, fill=False, ec="orange", lw=2,
                                label=f"closest approach ({1000 * res['min_clearance']:.1f} mm)"))
    ax.plot(s["x"], s["y"], "go", ms=8)
    ax.plot(world["goal"]["x"], world["goal"]["y"], "r*", ms=14)
    ax.set_xlim(-5.5, 5.5)
    ax.set_ylim(-5.5, 5.5)
    ax.set_aspect("equal")
    if res.get("success"):
        verdict = "success"
    elif res.get("true_goal_error") is not None:
        verdict = (f"FAIL: {res['outcome']}, {res.get('collisions', 0)} bumper hits, "
                   f"{res['true_goal_error']:.2f} m from goal")
    else:
        verdict = f"FAIL: {res.get('outcome')} {res.get('error', '')}"
    ax.set_title(f"seed {res['seed']}  {res['planner']} + {res['controller']}\n{verdict}", fontsize=10)
    ax.legend(loc="lower left", fontsize=7)
    fig.tight_layout()
    fig.savefig(out, dpi=110)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])