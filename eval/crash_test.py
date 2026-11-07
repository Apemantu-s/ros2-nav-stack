#!/usr/bin/env python3
import argparse
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_trial as rt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--speed", type=float, default=0.3)
    ap.add_argument("--seconds", type=float, default=25.0, help="sim seconds")
    a = ap.parse_args()

    work = "/tmp/crash_test"
    os.makedirs(work, exist_ok=True)
    subprocess.run([sys.executable, rt.MAKE_WORLD, str(a.seed), work + "/world.sdf", "--rtf", "4",
                    "--obstacles", "0"], check=True)
    world = json.load(open(work + "/world.json"))
    sim = subprocess.Popen(["ros2", "launch", "nav_sim", "sim.launch.py", f"world:={work}/world.sdf"],
                           stdout=open(work + "/launch.log", "w"), stderr=subprocess.STDOUT,
                           start_new_session=True)
    rclpy.init()
    rec = rt.Recorder()
    cmd_pub = rec.create_publisher(Twist, "/cmd_vel", 10)
    bumps = []
    rec.create_subscription(rt.Contacts, "/bumper",
                            lambda m: bumps.append(rec.now()) if m.contacts else None, 50)
    try:
        t0 = time.time()
        while not rec.gt and time.time() - t0 < 60:
            rclpy.spin_once(rec, timeout_sec=0.1)
        start = rec.now()
        cmd = Twist()
        cmd.linear.x = a.speed
        while rec.now() - start < a.seconds:
            cmd_pub.publish(cmd)
            rclpy.spin_once(rec, timeout_sec=0.05)

        t = start
        while t < rec.now():
            second = [p for p in rec.gt if t <= p[0] < t + 1.0]
            if second:
                clear, _ = rt.min_clearance(second, world["obstacles"])
                n = sum(1 for b in bumps if t <= b < t + 1.0)
                p = second[0]
                print(f"t={t - start:5.1f}  x={p[1]:+.2f} y={p[2]:+.2f}  "
                      f"clearance={clear:+.4f} m  bumper msgs={n}")
            t += 1.0
    finally:
        os.killpg(sim.pid, signal.SIGINT)
        time.sleep(2)
        os.killpg(sim.pid, signal.SIGKILL)
        rec.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()