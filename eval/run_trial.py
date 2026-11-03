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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

HERE = os.path.dirname(os.path.abspath(__file__))
MAKE_WORLD = os.path.join(HERE, "..", "src", "nav_sim", "scripts", "make_world.py")


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(q):
    return 2 * math.atan2(q.z, q.w)


class Recorder(Node):
    """subscribes to everything we score and just stores it in lists"""

    def __init__(self):
        super().__init__("trial_recorder", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.gt = []      
        self.ekf = []
        self.wheel = []
        self.create_subscription(Odometry, "/ground_truth", lambda m: self.save_pose(self.gt, m), 50)
        self.create_subscription(Odometry, "/ekf/odom", lambda m: self.save_pose(self.ekf, m), 50)
        self.create_subscription(Odometry, "/wheel/odom", lambda m: self.save_pose(self.wheel, m), 50)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def save_pose(self, lst, m):
        p = m.pose.pose
        lst.append((stamp_to_sec(m.header.stamp), p.position.x, p.position.y, yaw_of(p.orientation)))

    def drive(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)


def world_to_odom(start, x, y):
    """world -> odom frame (odom starts where the robot spawns)"""
    dx = x - start["x"]
    dy = y - start["y"]
    c = math.cos(-start["yaw"])
    s = math.sin(-start["yaw"])
    return c * dx - s * dy, s * dx + c * dy


def compare(est, gt_odom):
    """position / heading error of an estimate vs ground truth (both odom frame).
    ground truth is interpolated at the estimate's timestamps"""
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



PATTERN = [(3.0, 0.0, 0.0),
           (12.6, 0.15, 0.5), (12.6, 0.15, -0.5),
           (12.6, 0.15, 0.5), (12.6, 0.15, -0.5),
           (1.0, 0.0, 0.0)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--q-w", type=float, default=0.1)
    ap.add_argument("--use-imu", default="true")
    ap.add_argument("--rtf", type=float, default=4.0)
    ap.add_argument("--out", default="results/testqw")
    a = ap.parse_args()

    tag = f"{a.seed:04d}_qw{a.q_w}"
    os.makedirs(a.out, exist_ok=True)
    work = "/tmp/trial_" + tag
    os.makedirs(work, exist_ok=True)
    world_sdf = work + "/world.sdf"
    subprocess.run([sys.executable, MAKE_WORLD, str(a.seed), world_sdf, "--rtf", str(a.rtf),
                    "--obstacles", "0"], check=True)
    world = json.load(open(work + "/world.json"))
    start = world["start"]

    commands = [
        ["ros2", "launch", "nav_sim", "sim.launch.py", f"world:={world_sdf}"],
        ["ros2", "run", "nav_stack", "wheel_odom_node", "--ros-args", "-p", "use_sim_time:=true"],
        ["ros2", "run", "nav_stack", "ekf_node", "--ros-args", "-p", "use_sim_time:=true",
         "-p", f"q_w:={a.q_w}", "-p", f"use_imu:={a.use_imu}"],
    ]
    log = open(work + "/launch.log", "w")
    procs = [subprocess.Popen(c, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
             for c in commands]

    rclpy.init()
    rec = Recorder()
    result = {"seed": a.seed, "q_w": a.q_w, "use_imu": a.use_imu == "true"}
    try:
        wall_start = time.time()
        while (not rec.ekf or not rec.gt) and time.time() - wall_start < 60:
            rclpy.spin_once(rec, timeout_sec=0.1)
        if not rec.ekf:
            raise RuntimeError("stack did not start, see " + work + "/launch.log")

        t_start = rec.now()
        for seconds, v, w in PATTERN:
            t0 = rec.now()
            while rec.now() - t0 < seconds:
                rec.drive(v, w)
                rclpy.spin_once(rec, timeout_sec=0.05)
        rec.drive(0.0, 0.0)

        gt_odom = []
        for (t, x, y, yaw) in rec.gt:
            ox, oy = world_to_odom(start, x, y)
            gt_odom.append((t, ox, oy, yaw - start["yaw"]))
        ekf = [p for p in rec.ekf if p[0] >= t_start]
        wheel = [p for p in rec.wheel if p[0] >= t_start]
        result["ekf"] = compare(ekf, gt_odom)
        result["wheel_odom"] = compare(wheel, gt_odom)
    except Exception as ex:
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
    if "error" in result:
        print(f"seed {a.seed}: ERROR {result['error']}")
    else:
        print(f"seed {a.seed} q_w {a.q_w}: heading error at the end: "
              f"ekf {result['ekf']['yaw_final_deg']:.2f} deg, "
              f"wheels {result['wheel_odom']['yaw_final_deg']:.2f} deg")


if __name__ == "__main__":
    main()
