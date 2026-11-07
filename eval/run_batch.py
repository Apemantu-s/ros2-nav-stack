#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def parse_seeds(s):
    """'0-4,10,20-22' -> [0, 1, 2, 3, 4, 10, 20, 21, 22]"""
    out = []
    for part in s.split(","):
        if "-" in part:
            a, b = part.split("-")
            out += list(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


ap = argparse.ArgumentParser()
ap.add_argument("--seeds", default="0-199")
ap.add_argument("--planner", default="astar")
ap.add_argument("--controller", default="mpc")
ap.add_argument("--use-imu", default="true")
ap.add_argument("--obstacles", type=int, default=12)
ap.add_argument("--jobs", type=int, default=3)
ap.add_argument("--rtf", type=float, default=4.0)
ap.add_argument("--out", default="results/trials")
a = ap.parse_args()

todo = []
for seed in parse_seeds(a.seeds):
    tag = f"{seed:04d}_{a.planner}_{a.controller}" + ("" if a.use_imu == "true" else "_noimu")
    if not os.path.exists(os.path.join(a.out, tag + ".json")):
        todo.append(seed)
print(f"{len(todo)} trials to run with {a.jobs} jobs")

running = {}
started = {}
free_slots = list(range(a.jobs))
done = 0
t0 = time.time()
while todo or running:
    while todo and free_slots:
        slot = free_slots.pop(0)
        seed = todo.pop(0)
        cmd = [sys.executable, os.path.join(HERE, "run_trial.py"), "--seed", str(seed),
               "--planner", a.planner, "--controller", a.controller, "--use-imu", a.use_imu,
               "--rtf", str(a.rtf), "--obstacles", str(a.obstacles), "--slot", str(slot),
               "--out", a.out]
        running[slot] = (subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                          text=True), seed)
        started[slot] = time.time()

    time.sleep(0.5)

    for slot, (p, seed) in list(running.items()):
        if p.poll() is None and time.time() - started[slot] > 900:
            p.kill()
            p.wait()
        if p.poll() is not None:
            lines = p.stdout.read().strip().splitlines()
            done += 1
            last = lines[-1] if lines else f"seed {seed}: no output"
            print(f"[{done}] slot {slot}: {last}  ({time.time() - t0:.0f} s)", flush=True)
            del running[slot]
            free_slots.append(slot)

print(f"done in {time.time() - t0:.0f} s")