#!/usr/bin/env python3
import collections
import glob
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

root = sys.argv[1] if len(sys.argv) > 1 else "results"
figdir = os.path.join(root, "figures")
os.makedirs(figdir, exist_ok=True)

CONFIGS = ["astar + mpc", "astar + pure_pursuit", "rrtstar + mpc", "rrtstar + pure_pursuit"]
REASONS = ["error", "never_moved", "timeout", "bumper", "touched", "reached_but_off"]


def wilson(k, n, z=1.96):
    """95% interval for a success rate. better than p +- 1.96*sqrt(p(1-p)/n) near
    0% or 100%, where that one can go past 100%"""
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (c - h, c + h)


def why_failed(r):
    """one reason per failed trial, the first one that applies"""
    if r.get("outcome") not in ("reached", "timeout"):
        return "error"
    if r.get("never_moved"):
        return "never_moved"
    if r["outcome"] == "timeout":
        return "timeout"
    if r.get("collisions", 0) > 0:
        return "bumper"
    if r.get("touched"):
        return "touched"
    return "reached_but_off"


def median(xs):
    xs = [x for x in xs if x is not None]
    return float(np.median(xs)) if xs else None


def mean(xs):
    xs = [x for x in xs if x is not None]
    return float(np.mean(xs)) if xs else None


def fmt(x, spec, unit=""):
    return "-" if x is None else f"{x:{spec}}{unit}"


trials = []
for f in sorted(glob.glob(os.path.join(root, "trials*", "*.json"))):
    r = json.load(open(f))
    r["set"] = os.path.basename(os.path.dirname(f))
    r["file"] = f[:-5]
    r["config"] = f"{r['planner']} + {r['controller']}"
    r["success"] = bool(r.get("success"))
    r["reason"] = None if r["success"] else why_failed(r)
    trials.append(r)
if not trials:
    sys.exit(f"no trial files in {root}/trials*/")

groups = collections.defaultdict(list)
for r in trials:
    groups[(r["set"], r["config"])].append(r)
order = sorted(groups, key=lambda k: (k[0], CONFIGS.index(k[1]) if k[1] in CONFIGS else 99))

summary = {"navigation": {}}
lines = ["# Results", "",
         "Made by `eval/summarize.py` from the raw results. Don't edit by hand.", ""]

lines += ["## Navigation, one goal per world", "",
          "| set | planner + controller | worlds | success | 95% CI | bumper | touched | timeout "
          "| never moved | reached but off | error | median time to goal | median clearance | tracking RMS |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
for key in order:
    g = groups[key]
    n, k = len(g), sum(r["success"] for r in g)
    lo, hi = wilson(k, n)
    ok = [r for r in g if r["success"]]
    reasons = collections.Counter(r["reason"] for r in g if r["reason"])
    s = {"worlds": n, "successes": k, "success_rate": k / n, "success_ci95": [lo, hi],
         "failures": {x: reasons.get(x, 0) for x in REASONS},
         "time_to_goal_median_s": median(r.get("time_to_goal") for r in ok),
         "path_ratio_median": median(r["distance_driven"] / r["straight_line"] for r in ok),
         "min_clearance_median_m": median(r.get("min_clearance") for r in g),
         "tracking_rms_mean_m": mean(r.get("tracking_error_rms") for r in g),
         "plan_time_ms_mean": mean(r.get("plan_time_ms_mean") for r in g)}
    summary["navigation"][f"{key[0]} | {key[1]}"] = s
    fl = s["failures"]
    lines.append(f"| {key[0]} | {key[1]} | {n} | {100 * k / n:.1f}% | {100 * lo:.1f} to {100 * hi:.1f}% | "
                 f"{fl['bumper']} | {fl['touched']} | {fl['timeout']} | {fl['never_moved']} | "
                 f"{fl['reached_but_off']} | {fl['error']} | {fmt(s['time_to_goal_median_s'], '.1f', ' s')} | "
                 f"{fmt(s['min_clearance_median_m'], '.3f', ' m')} | {fmt(s['tracking_rms_mean_m'], '.3f', ' m')} |")
lines += ["", "Each failure is counted once, under the first reason that applies, in the order "
          "error, never moved, timeout, bumper, touched (clearance under 5 mm), reached but off "
          "(more than 0.5 m from the goal by ground truth). Median time is over successful trials only.", ""]

main = [r for r in trials if r["set"] == "trials"]
by_seed = collections.defaultdict(dict)
for r in main:
    if r.get("tracking_error_rms") is not None:
        by_seed[r["config"]][r["seed"]] = r["tracking_error_rms"]
pairs = {}
for planner in ["astar", "rrtstar"]:
    pp, mpc = by_seed.get(f"{planner} + pure_pursuit", {}), by_seed.get(f"{planner} + mpc", {})
    seeds = sorted(set(pp) & set(mpc))
    if seeds:
        a = np.array([pp[s] for s in seeds])
        b = np.array([mpc[s] for s in seeds])
        pairs[planner] = {"worlds": len(seeds), "pure_pursuit_mean_m": float(a.mean()),
                          "mpc_mean_m": float(b.mean()), "mpc_lower_share": float((b < a).mean())}
summary["controller_pairs"] = pairs
if pairs:
    lines += ["## Controllers on the same worlds (tracking error RMS, main set)", "",
              "| planner | worlds | pure pursuit | MPC | MPC lower in |", "|---|---|---|---|---|"]
    for p, v in pairs.items():
        lines.append(f"| {p} | {v['worlds']} | {v['pure_pursuit_mean_m']:.3f} m | {v['mpc_mean_m']:.3f} m | "
                     f"{100 * v['mpc_lower_share']:.0f}% of worlds |")
    lines.append("")

loc = [r for r in trials if (r.get("ekf") or {}).get("final") is not None
       and (r.get("wheel_odom") or {}).get("final") is not None]
if loc:
    ekf = np.array([r["ekf"]["final"] for r in loc])
    whl = np.array([r["wheel_odom"]["final"] for r in loc])
    ekf_rmse = [r["ekf"]["rmse"] for r in loc]
    whl_rmse = [r["wheel_odom"]["rmse"] for r in loc]
    ekf_yaw = [abs(r["ekf"]["yaw_final_deg"]) for r in loc]
    whl_yaw = [abs(r["wheel_odom"]["yaw_final_deg"]) for r in loc]
    ls = {"trials": len(loc),
          "ekf_final_median_m": float(np.median(ekf)), "ekf_final_p90_m": float(np.percentile(ekf, 90)),
          "wheel_final_median_m": float(np.median(whl)), "wheel_final_p90_m": float(np.percentile(whl, 90)),
          "ekf_rmse_median_m": median(ekf_rmse), "wheel_rmse_median_m": median(whl_rmse),
          "ekf_yaw_final_median_deg": median(ekf_yaw), "wheel_yaw_final_median_deg": median(whl_yaw),
          "ekf_better_share": float((ekf < whl).mean())}
    summary["localization"] = ls
    lines += ["## Localization (every trial)", "",
              "| | final position error, median | 90th percentile | trajectory RMSE, median | final heading error, median |",
              "|---|---|---|---|---|",
              f"| wheel odometry | {ls['wheel_final_median_m']:.3f} m | {ls['wheel_final_p90_m']:.3f} m | "
              f"{ls['wheel_rmse_median_m']:.3f} m | {ls['wheel_yaw_final_median_deg']:.1f} deg |",
              f"| EKF (wheels + IMU) | {ls['ekf_final_median_m']:.3f} m | {ls['ekf_final_p90_m']:.3f} m | "
              f"{ls['ekf_rmse_median_m']:.3f} m | {ls['ekf_yaw_final_median_deg']:.1f} deg |", "",
              f"The EKF ended closer to the truth than wheel odometry in {100 * ls['ekf_better_share']:.1f}% "
              f"of {ls['trials']} trials.", ""]

    fig, ax = plt.subplots(figsize=(5, 5))
    ax.scatter(whl, ekf, s=6, alpha=0.5)
    m = max(whl.max(), ekf.max())
    ax.plot([0, m], [0, m], "k--", lw=1)
    ax.set_xlabel("wheel odometry final error (m)")
    ax.set_ylabel("EKF final error (m)")
    ax.set_title("below the line = EKF better")
    fig.tight_layout()
    fig.savefig(os.path.join(figdir, "ekf_vs_wheel.png"), dpi=110)
    plt.close(fig)

bench_file = os.path.join(root, "bench.csv")
if os.path.exists(bench_file):
    b = np.genfromtxt(bench_file, delimiter=",", names=True, dtype=None, encoding="utf-8")
    astar_len = {int(r["case"]): float(r["length"]) for r in b if r["planner"] == "astar" and r["success"] == 1}
    bs = {}
    for p in dict.fromkeys(b["planner"]):
        g = b[b["planner"] == p]
        ok = g[g["success"] == 1]
        ratio = [float(r["length"]) / astar_len[int(r["case"])] for r in ok if int(r["case"]) in astar_len]
        bs[str(p)] = {"runs": int(len(g)), "success_rate": float(g["success"].mean()),
                      "time_ms_median": float(np.median(g["time_ms"])),
                      "length_m_mean": float(ok["length"].mean()) if len(ok) else None,
                      "length_vs_astar_median": median(ratio)}
    summary["planner_benchmark"] = bs
    lines += ["## Planners offline, on the true maps", "",
              "| planner | runs | success | median time | mean length | length / A* (median) |",
              "|---|---|---|---|---|---|"]
    for p, s in bs.items():
        lines.append(f"| {p} | {s['runs']} | {100 * s['success_rate']:.1f}% | {s['time_ms_median']:.2f} ms | "
                     f"{fmt(s['length_m_mean'], '.2f', ' m')} | {fmt(s['length_vs_astar_median'], '.3f')} |")
    lines.append("")

mapping_file = os.path.join(root, "mapping.json")
if os.path.exists(mapping_file):
    mp = json.load(open(mapping_file))
    summary["mapping"] = mp
    lines += ["## Mapping: our mapper vs slam_toolbox", "",
              "| mapper | runs | precision (occupied cells within 10 cm of a real obstacle) | coverage (obstacle edges found) |",
              "|---|---|---|---|"]
    for k, v in mp.items():
        lines.append(f"| {k} | {v['runs']} | {100 * v['precision']:.1f}% | {100 * v['coverage']:.1f}% |")
    lines.append("")

rank = {k: i for i, k in enumerate(order)}
fails = sorted((r for r in trials if not r["success"]), key=lambda r: (rank[(r["set"], r["config"])], r["seed"]))
summary["failures"] = [{"file": r["file"], "set": r["set"], "config": r["config"], "seed": r["seed"],
                        "reason": r["reason"], "true_goal_error_m": r.get("true_goal_error"),
                        "min_clearance_m": r.get("min_clearance")} for r in fails]
lines += [f"## Every failure ({len(fails)})", "",
          "| set | planner + controller | seed | reason | true goal error | min clearance | plot it with |",
          "|---|---|---|---|---|---|---|"]
for r in fails:
    lines.append(f"| {r['set']} | {r['config']} | {r['seed']} | {r['reason']} | "
                 f"{fmt(r.get('true_goal_error'), '.2f', ' m')} | {fmt(r.get('min_clearance'), '.4f', ' m')} | "
                 f"`python3 eval/plot_trial.py {r['file']} fail.png` |")
lines.append("")

keys = list(summary["navigation"])
rates = [100 * summary["navigation"][k]["success_rate"] for k in keys]
err = [[100 * (summary["navigation"][k]["success_rate"] - summary["navigation"][k]["success_ci95"][0]) for k in keys],
       [100 * (summary["navigation"][k]["success_ci95"][1] - summary["navigation"][k]["success_rate"]) for k in keys]]
fig, ax = plt.subplots(figsize=(7, 4))
ax.bar(range(len(keys)), rates, yerr=err, capsize=4)
ax.set_xticks(range(len(keys)), [k.replace(" | ", "\n").replace(" + ", "+") for k in keys], fontsize=7)
ax.set_ylabel("success rate (%), with 95% interval")
ax.set_ylim(min(80, min(rates) - 5), 101)
fig.tight_layout()
fig.savefig(os.path.join(figdir, "success.png"), dpi=110)
plt.close(fig)

cfgs = [c for c in CONFIGS if by_seed.get(c)]
if cfgs:
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.boxplot([list(by_seed[c].values()) for c in cfgs], showfliers=False)
    ax.set_xticks(range(1, len(cfgs) + 1), [c.replace(" + ", "\n") for c in cfgs])
    ax.set_ylabel("cross track error RMS (m)")
    fig.tight_layout()
    fig.savefig(os.path.join(figdir, "tracking_error.png"), dpi=110)
    plt.close(fig)

fig, ax = plt.subplots(figsize=(6, 4))
for planner in ["astar", "rrtstar"]:
    c = [r["min_clearance"] for r in main if r["planner"] == planner and r.get("min_clearance") is not None]
    if c:
        ax.hist(np.clip(c, -0.02, 0.6), bins=np.linspace(-0.02, 0.6, 63), alpha=0.6,
                label=f"{planner} ({len(c)} trials)")
ax.axvline(0.005, color="k", ls="--", lw=1, label="5 mm = touched")
ax.set_xlabel("closest the robot got to an obstacle (m)")
ax.set_ylabel("trials")
ax.legend()
fig.tight_layout()
fig.savefig(os.path.join(figdir, "clearance.png"), dpi=110)
plt.close(fig)

json.dump(summary, open(os.path.join(root, "summary.json"), "w"), indent=1)
open(os.path.join(root, "summary.md"), "w").write("\n".join(lines) + "\n")
print("\n".join(lines))
print(f"wrote {root}/summary.json, {root}/summary.md and {len(os.listdir(figdir))} figures in {figdir}/")