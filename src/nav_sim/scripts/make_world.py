#!/usr/bin/env python3
"""random world generator: walls + random boxes and cylinders

usage: make_world.py SEED OUT.sdf [--size 10] [--obstacles 12]
also writes OUT.json (start, goal, obstacles) for scoring later
same seed = same world
"""
import argparse
import json
import math
import random

HEADER = """<?xml version="1.0"?>
<sdf version="1.9">
<world name="nav_world">
  <physics name="1ms" type="ignored">
    <max_step_size>0.004</max_step_size>
    <real_time_factor>{rtf}</real_time_factor>
  </physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
  <light type="directional" name="sun">
    <cast_shadows>false</cast_shadows>
    <pose>0 0 10 0 0 0</pose>
    <direction>-0.5 0.1 -0.9</direction>
  </light>
  <model name="ground">
    <static>true</static>
    <link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        <material><ambient>0.8 0.8 0.8 1</ambient></material></visual>
    </link>
  </model>
"""

BOX = """  <model name="{name}">
    <static>true</static>
    <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
    <link name="link">
      <collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        <material><ambient>0.6 0.3 0.2 1</ambient></material></visual>
    </link>
  </model>
"""

CYL = """  <model name="{name}">
    <static>true</static>
    <pose>{x:.3f} {y:.3f} 0.4 0 0 0</pose>
    <link name="link">
      <collision name="c"><geometry><cylinder><radius>{r:.3f}</radius><length>0.8</length></cylinder></geometry></collision>
      <visual name="v"><geometry><cylinder><radius>{r:.3f}</radius><length>0.8</length></cylinder></geometry>
        <material><ambient>0.2 0.6 0.2 1</ambient></material></visual>
    </link>
  </model>
"""

ROBOT = """  <include>
    <uri>model://diffbot</uri>
    <name>diffbot</name>
    <pose>{x:.3f} {y:.3f} 0.05 0 0 {yaw:.3f}</pose>
  </include>
"""


def box_corners(o):
    c, s = math.cos(o["yaw"]), math.sin(o["yaw"])
    hx, hy = o["sx"] / 2, o["sy"] / 2
    return [(o["x"] + c * dx - s * dy, o["y"] + s * dx + c * dy)
            for dx, dy in [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]]


def dist_to_obstacle(px, py, o):
    """distance from point to obstacle edge, negative if inside"""
    if o["type"] == "cyl":
        return math.hypot(px - o["x"], py - o["y"]) - o["r"]
    # box: rotate point into the box frame
    c, s = math.cos(-o["yaw"]), math.sin(-o["yaw"])
    dx, dy = px - o["x"], py - o["y"]
    lx, ly = c * dx - s * dy, s * dx + c * dy
    qx, qy = abs(lx) - o["sx"] / 2, abs(ly) - o["sy"] / 2
    outside = math.hypot(max(qx, 0), max(qy, 0))
    inside = min(max(qx, qy), 0)
    return outside + inside


def clearance(px, py, obstacles):
    return min(dist_to_obstacle(px, py, o) for o in obstacles)


def make(seed, size, n_obs):
    rng = random.Random(seed)
    half = size / 2
    obstacles = []
    t = 0.2 
    for name, x, y, sx, sy in [("wall_n", 0, half, size + t, t), ("wall_s", 0, -half, size + t, t),
                               ("wall_e", half, 0, t, size + t), ("wall_w", -half, 0, t, size + t)]:
        obstacles.append(dict(type="box", name=name, x=x, y=y, sx=sx, sy=sy, sz=0.8, yaw=0.0))

    while True:
        sx_, sy_ = rng.uniform(-half + 1, half - 1), rng.uniform(-half + 1, half - 1)
        gx, gy = rng.uniform(-half + 1, half - 1), rng.uniform(-half + 1, half - 1)
        if math.hypot(gx - sx_, gy - sy_) > 0.6 * size:
            break

    tries = 0
    while len(obstacles) < 4 + n_obs and tries < 2000:
        tries += 1
        x, y = rng.uniform(-half + 0.5, half - 0.5), rng.uniform(-half + 0.5, half - 0.5)
        if rng.random() < 0.65:
            o = dict(type="box", name=f"box_{len(obstacles)}", x=x, y=y,
                     sx=rng.uniform(0.3, 2.0), sy=rng.uniform(0.3, 1.0), sz=0.8,
                     yaw=rng.uniform(-math.pi, math.pi))
        else:
            o = dict(type="cyl", name=f"cyl_{len(obstacles)}", x=x, y=y, r=rng.uniform(0.15, 0.5))
        if dist_to_obstacle(sx_, sy_, o) < 0.7 or dist_to_obstacle(gx, gy, o) < 0.7:
            continue
        obstacles.append(o)

    return dict(seed=seed, size=size,
                start=dict(x=sx_, y=sy_, yaw=rng.uniform(-math.pi, math.pi)),
                goal=dict(x=gx, y=gy), obstacles=obstacles)


def to_sdf(w, rtf):
    out = [HEADER.format(rtf=rtf)]
    for o in w["obstacles"]:
        if o["type"] == "box":
            out.append(BOX.format(z=o["sz"] / 2, **o))
        else:
            out.append(CYL.format(**o))
    s = w["start"]
    out.append(ROBOT.format(x=s["x"], y=s["y"], yaw=s["yaw"]))
    out.append("</world>\n</sdf>\n")
    return "".join(out)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("seed", type=int)
    ap.add_argument("out")
    ap.add_argument("--size", type=float, default=10.0)
    ap.add_argument("--obstacles", type=int, default=12)
    ap.add_argument("--rtf", type=float, default=0.0, help="0 = run as fast as possible")
    a = ap.parse_args()
    w = make(a.seed, a.size, a.obstacles)
    with open(a.out, "w") as f:
        f.write(to_sdf(w, a.rtf))
    with open(a.out.rsplit(".", 1)[0] + ".json", "w") as f:
        json.dump(w, f, indent=1)
