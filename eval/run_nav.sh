#!/bin/bash

SEED=${1:?usage: run_nav.sh SEED [mpc|pure_pursuit] [astar|rrtstar]}
CTRL=${2:-mpc}
PLANNER=${3:-astar}
TIMEOUT=120

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export GZ_IP=127.0.0.1
OUT=/tmp/run_nav; mkdir -p $OUT; cd /ws
ros2 daemon stop >/dev/null 2>&1

python3 src/nav_sim/scripts/make_world.py $SEED $OUT/w.sdf > /dev/null
ros2 launch nav_sim sim.launch.py world:=$OUT/w.sdf > $OUT/sim.log 2>&1 &
sleep 8
ros2 launch nav_stack nav.launch.py controller:=$CTRL planner:=$PLANNER > $OUT/nav.log 2>&1 &
sleep 5

timeout $TIMEOUT ros2 topic echo /nav/status std_msgs/msg/String \
  --qos-durability transient_local --qos-reliability reliable --field data > $OUT/status.log 2>&1 &
timeout $TIMEOUT ros2 topic echo /nav/tracking_error std_msgs/msg/Float64 --field data > $OUT/err.log 2>&1 &
sleep 2

python3 eval/send_goal.py $OUT/w.json
for i in $(seq 1 $((TIMEOUT * 10))); do
  grep -q reached $OUT/status.log && break
  sleep 0.1
done

echo "world $SEED, $PLANNER + $CTRL"
if grep -q reached $OUT/status.log; then
  grep -o "goal reached in .*" $OUT/nav.log
else
  echo "did NOT reach the goal within $TIMEOUT s (real time)"
fi
python3 - $OUT/err.log <<'PY'
import sys
v = []
for line in open(sys.argv[1]):
    try:
        v.append(float(line))
    except ValueError:
        pass   # '---' separators and warnings
if v:
    print(f"tracking error: mean {sum(v)/len(v):.3f} m, max {max(v):.3f} m ({len(v)} ticks)")
PY

pkill -f "ros2 launch|ros2 topic|gz sim|parameter_bridge|lib/nav_stack/|static_transform"
sleep 1