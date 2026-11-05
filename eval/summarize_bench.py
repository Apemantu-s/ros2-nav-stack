#!/usr/bin/env python3
"""summarize bench_planners output

usage: python3 eval/summarize_bench.py results/bench.csv
"""
import csv
import statistics
import sys
from collections import defaultdict

rows = list(csv.DictReader(open(sys.argv[1])))

astar_len = {r["case"]: float(r["length"]) for r in rows
             if r["planner"] == "astar" and r["success"] == "1"}

by_planner = defaultdict(list)
for r in rows:
    by_planner[r["planner"]].append(r)

print(f"{'planner':<14}{'success':>9}{'median ms':>11}{'median exp':>12}{'length / A*':>13}")
for name, rs in by_planner.items():
    ok = [r for r in rs if r["success"] == "1"]
    ratios = [float(r["length"]) / astar_len[r["case"]] for r in ok if r["case"] in astar_len]
    print(f"{name:<14}{100 * len(ok) / len(rs):>8.1f}%"
          f"{statistics.median(float(r['time_ms']) for r in ok):>11.2f}"
          f"{statistics.median(int(r['expansions']) for r in ok):>12.0f}"
          f"{statistics.median(ratios):>13.3f}")