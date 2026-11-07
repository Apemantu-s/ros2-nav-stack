#!/usr/bin/env python3
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Float64, Float64MultiArray, String

HERE = os.path.dirname(os.path.abspath(__file__))
MAKE_WORLD = os.path.join(HERE, "..", "src", "nav_sim", "scripts", "make_world.py")

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(q):
    return 2 * math.atan2(q.z, q.w)


class Recorder(Node):

    def __init__(self):
        super().__init__("trial_recorder", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.gt = []
        self.ekf = []
        self.wheel = []
        self.tracking_errors = []
        self.plan_stats = []
        self.collisions = 0
        self.last_contact_time = -100.0
        self.status = None
        self.status_time = None
        self.map = None
        self.slam_map = None

        self.create_subscription(Odometry, "/ground_truth", lambda m: self.save_pose(self.gt, m), 50)
        self.create_subscription(Odometry, "/ekf/odom", lambda m: self.save_pose(self.ekf, m), 50)
        self.create_subscription(Odometry, "/wheel/odom", lambda m: self.save_pose(self.wheel, m), 50)
        self.create_subscription(Contacts, "/bumper", self.on_bumper, 50)
        self.create_subscription(Float64, "/nav/tracking_error",
                                 lambda m: self.tracking_errors.append(m.data), 50)
        self.create_subscription(Float64MultiArray, "/planner/stats",
                                 lambda m: self.plan_stats.append(list(m.data)), 50)
        self.create_subscription(String, "/nav/status", self.on_status, LATCHED)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, LATCHED)
        self.create_subscription(OccupancyGrid, "/slam_map", self.on_slam_map, LATCHED)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def save_pose(self, lst, m):
        p = m.pose.pose
        lst.append((stamp_to_sec(m.header.stamp), p.position.x, p.position.y, yaw_of(p.orientation)))

    def on_bumper(self, m):
        if len(m.contacts) > 0:
            t = self.now()
            if t - self.last_contact_time > 1.0:
                self.collisions += 1
            self.last_contact_time = t

    def on_status(self, m):
        self.status = m.data
        self.status_time = self.now()

    def on_map(self, m):
        self.map = m

    def on_slam_map(self, m):
        self.slam_map = m

    def send_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        t0 = time.time()
        while self.goal_pub.get_subscription_count() < 2 and time.time() - t0 < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.goal_pub.publish(goal)


def world_to_odom(start, x, y):
    dx = x - start["x"]
    dy = y - start["y"]
    c = math.cos(-start["yaw"])
    s = math.sin(-start["yaw"])
    return c * dx - s * dy, s * dx + c * dy


def compare(est, gt_odom):
    if len(est) < 2 or len(gt_odom) < 2:
        return None
    est = np.array(est)
    gt = np.array(gt_odom)
    gx = np.interp(est[:, 0], gt[:, 0], gt[:, 1])
    gy = np.interp(est[:, 0], gt[:, 0], gt[:, 2])
    gyaw = np.interp(est[:, 0], gt[:, 0], np.unwrap(gt[:, 3]))
    pos_err = np.hypot(est[:, 1] - gx, est[:, 2] - gy)
    yaw_err = np.abs(np.arctan2(np.sin(est[:, 3] - gyaw), np.cos(est[:, 3] - gyaw)))
    return {"rmse": float(np.sqrt(np.mean(pos_err ** 2))), "final": float(pos_err[-1]),
            "max": float(pos_err.max()),
            "yaw_rmse_deg": float(np.degrees(np.sqrt(np.mean(yaw_err ** 2)))),
            "yaw_final_deg": float(np.degrees(yaw_err[-1]))}


def robot_outline():
    pts = []

    def rect(x0, x1, y0, y1):
        for x in np.arange(x0, x1 + 1e-9, 0.02):
            pts.extend([(x, y0), (x, y1)])
        for y in np.arange(y0, y1 + 1e-9, 0.02):
            pts.extend([(x0, y), (x1, y)])

    rect(-0.18, 0.10, -0.13, 0.13)
    rect(-0.05, 0.05, 0.135, 0.175)
    rect(-0.05, 0.05, -0.175, -0.135)
    for a in np.arange(0, 2 * math.pi, 0.5):
        pts.append((0.08 + 0.035 * math.cos(a), 0.035 * math.sin(a)))
    return np.array(pts)


def min_clearance(gt, obstacles):
    gt = np.array(gt)[::2]
    out = robot_outline()
    c, s = np.cos(gt[:, 3])[:, None], np.sin(gt[:, 3])[:, None]
    px = gt[:, 1][:, None] + c * out[:, 0] - s * out[:, 1]
    py = gt[:, 2][:, None] + s * out[:, 0] + c * out[:, 1]
    best = np.full(px.shape, np.inf)
    for o in obstacles:
        if o["type"] == "cyl":
            d = np.hypot(px - o["x"], py - o["y"]) - o["r"]
        else:
            co, so = math.cos(-o["yaw"]), math.sin(-o["yaw"])
            dx, dy = px - o["x"], py - o["y"]
            qx = np.abs(co * dx - so * dy) - o["sx"] / 2
            qy = np.abs(so * dx + co * dy) - o["sy"] / 2
            d = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0)
        best = np.minimum(best, d)
    per_pose = best.min(axis=1)
    i = int(per_pose.argmin())
    return float(per_pose[i]), float(gt[i, 0])


def kill_slot(partition):
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            env = open(f"/proc/{pid}/environ", "rb").read().split(b"\0")
        except OSError:
            continue
        if f"GZ_PARTITION={partition}".encode() in env:
            try:
                os.kill(int(pid), signal.SIGKILL)
            except OSError:
                pass


def save_grid(m, path):
    """the map as a numpy file: the grid, plus what you need to turn cells into metres"""
    np.savez_compressed(path, grid=np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width),
                        resolution=m.info.resolution,
                        origin=np.array([m.info.origin.position.x, m.info.origin.position.y]),
                        frame=m.header.frame_id)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--planner", default="astar")
    ap.add_argument("--controller", default="mpc")
    ap.add_argument("--use-imu", default="true")
    ap.add_argument("--obstacles", type=int, default=12)
    ap.add_argument("--rtf", type=float, default=4.0)
    ap.add_argument("--timeout", type=float, default=150.0, help="sim seconds")
    ap.add_argument("--out", default="results/trials")
    ap.add_argument("--slot", type=int, default=0, help="parallel slot, picks domain id and partition")
    ap.add_argument("--return-home", action="store_true", help="drive back to the start after the goal")
    ap.add_argument("--slam", action="store_true", help="also run slam_toolbox and save its map")
    a = ap.parse_args()

    tag = f"{a.seed:04d}_{a.planner}_{a.controller}"
    if a.use_imu != "true":
        tag += "_noimu"
    if a.return_home:
        tag += "_home"
    if a.slam:
        tag += "_slam"
    os.makedirs(a.out, exist_ok=True)
    work = "/tmp/trial_" + tag
    os.makedirs(work, exist_ok=True)
    world_sdf = work + "/world.sdf"
    subprocess.run([sys.executable, MAKE_WORLD, str(a.seed), world_sdf, "--rtf", str(a.rtf),
                    "--obstacles", str(a.obstacles)], check=True)
    world = json.load(open(work + "/world.json"))
    start = world["start"]

    if a.slot >= 0:
        os.environ["ROS_DOMAIN_ID"] = str(10 + a.slot)
        os.environ["GZ_PARTITION"] = f"trial{a.slot}"
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        kill_slot(os.environ["GZ_PARTITION"])

    commands = [
        ["ros2", "launch", "nav_sim", "sim.launch.py", f"world:={world_sdf}"],
        ["ros2", "launch", "nav_stack", "nav.launch.py", f"planner:={a.planner}",
         f"controller:={a.controller}", f"use_imu:={a.use_imu}"],
    ]
    if a.slam:
        commands.append(["ros2", "launch", "nav_stack", "slam.launch.py"])
    log = open(work + "/launch.log", "w")
    procs = [subprocess.Popen(c, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
             for c in commands]

    rclpy.init()
    rec = Recorder()
    result = {"seed": a.seed, "planner": a.planner, "controller": a.controller,
              "use_imu": a.use_imu == "true", "obstacles": a.obstacles}
    wall_start = time.time()
    wall_limit = a.timeout * 3 + 120

    def spin_for(sim_seconds):
        t0 = rec.now()
        while rec.now() - t0 < sim_seconds and time.time() - wall_start < wall_limit:
            rclpy.spin_once(rec, timeout_sec=0.05)

    def drive_until_reached():
        """wait for a "reached" that comes after this call. returns reached or timeout"""
        t0 = rec.now()
        while rec.now() - t0 < a.timeout and time.time() - wall_start < wall_limit:
            rclpy.spin_once(rec, timeout_sec=0.05)
            if rec.status == "reached" and rec.status_time > t0:
                return "reached"
        return "timeout"

    try:
        while (not rec.ekf or not rec.gt) and time.time() - wall_start < 60:
            rclpy.spin_once(rec, timeout_sec=0.1)
        if not rec.ekf:
            raise RuntimeError("stack did not start, see " + work + "/launch.log")
        spin_for(3.0)

        t_goal = rec.now()
        gx, gy = world_to_odom(start, world["goal"]["x"], world["goal"]["y"])
        rec.send_goal(gx, gy)
        outcome = drive_until_reached()
        reach_time = rec.status_time

        if a.return_home and outcome == "reached":
            rec.send_goal(0.0, 0.0)
            result["home_outcome"] = drive_until_reached()
            last = rec.gt[-1]
            result["home_true_error"] = math.hypot(last[1] - start["x"], last[2] - start["y"])
        if a.slam:
            spin_for(3.0)
        t_end = rec.now()
        if outcome == "timeout" and t_end - t_goal < 1.0:
            raise RuntimeError("sim clock stuck")

        gt = np.array(rec.gt)
        if outcome == "reached":
            final_x = np.interp(reach_time, gt[:, 0], gt[:, 1])
            final_y = np.interp(reach_time, gt[:, 0], gt[:, 2])
        else:
            final_x, final_y = gt[-1, 1], gt[-1, 2]
        true_goal_err = math.hypot(final_x - world["goal"]["x"], final_y - world["goal"]["y"])

        gt_odom = []
        for (t, x, y, yaw) in rec.gt:
            ox, oy = world_to_odom(start, x, y)
            gt_odom.append((t, ox, oy, yaw - start["yaw"]))
        driven = float(np.sum(np.hypot(np.diff(gt[:, 1]), np.diff(gt[:, 2]))))
        ekf_moving = [p for p in rec.ekf if p[0] >= t_goal]
        wheel_moving = [p for p in rec.wheel if p[0] >= t_goal]
        clear, clear_t = min_clearance([p for p in rec.gt if p[0] >= t_goal], world["obstacles"])
        errs = rec.tracking_errors
        plans = rec.plan_stats

        result.update({
            "outcome": outcome,
            "never_moved": outcome == "timeout" and driven < 0.1,
            "collisions": rec.collisions,
            "min_clearance": clear,
            "min_clearance_time": clear_t - t_goal,
            "touched": clear < 0.005,
            "success": (outcome == "reached" and true_goal_err < 0.5 and rec.collisions == 0
                        and clear >= 0.005),
            "reached_but_off": outcome == "reached" and true_goal_err >= 0.5,
            "time_to_goal": (reach_time - t_goal) if outcome == "reached" else None,
            "sim_time": t_end - t_goal,
            "wall_time": time.time() - wall_start,
            "true_goal_error": true_goal_err,
            "straight_line": math.hypot(world["goal"]["x"] - start["x"], world["goal"]["y"] - start["y"]),
            "distance_driven": driven,
            "ekf": compare(ekf_moving, gt_odom),
            "wheel_odom": compare(wheel_moving, gt_odom),
            "tracking_error_mean": float(np.mean(errs)) if errs else None,
            "tracking_error_rms": float(np.sqrt(np.mean(np.square(errs)))) if errs else None,
            "tracking_error_max": float(np.max(errs)) if errs else None,
            "plans": len(plans),
            "plan_time_ms_mean": float(np.mean([p[1] for p in plans])) if plans else None,
            "first_plan_length": plans[0][2] if plans else None,
        })

        times = np.arange(t_goal, t_end, 0.2)
        g, e, w = np.array(gt_odom), np.array(rec.ekf), np.array(rec.wheel)
        cols = [times - t_goal]
        for arr in [g, e, w]:
            cols.append(np.interp(times, arr[:, 0], arr[:, 1]))
            cols.append(np.interp(times, arr[:, 0], arr[:, 2]))
        np.savetxt(f"{a.out}/{tag}_traj.csv", np.column_stack(cols), delimiter=",", fmt="%.3f",
                   header="t,gt_x,gt_y,ekf_x,ekf_y,wheel_x,wheel_y", comments="")

        if rec.map is not None:
            save_grid(rec.map, f"{a.out}/{tag}_map.npz")
        if rec.slam_map is not None:
            save_grid(rec.slam_map, f"{a.out}/{tag}_slammap.npz")
    except Exception as ex:
        result.setdefault("outcome", "error")
        result["error"] = repr(ex)
    finally:
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        time.sleep(2)
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        rec.destroy_node()
        rclpy.shutdown()

    with open(f"{a.out}/{tag}.json", "w") as f:
        json.dump(result, f, indent=1)
    print(json.dumps({k: result.get(k) for k in ["seed", "outcome", "success", "collisions", "min_clearance", "time_to_goal", "true_goal_error"]}))


if __name__ == "__main__":
    main()