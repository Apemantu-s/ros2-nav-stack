#!/bin/bash
set -e
OUT=${1:-results}
J=${JOBS:-3}
if [ -n "$QUICK" ]; then
  MAIN=0-1; HARD=1000-1001; MAPPING="0 1"
else
  MAIN=0-199; HARD=1000-1099; MAPPING=$(seq 0 19)
fi
echo "$(date +%T) start, output in $OUT, $J jobs"

# 1. main set: 200 random worlds, 12 obstacles, all four planner + controller combos
for planner in astar rrtstar; do
  for controller in mpc pure_pursuit; do
    echo "$(date +%T) main set: $planner + $controller"
    python3 eval/run_batch.py --seeds $MAIN --planner $planner --controller $controller \
      --jobs $J --out $OUT/trials
  done
done

# 2. hard set: 100 different worlds with 20 obstacles, A* with both controllers
for controller in mpc pure_pursuit; do
  echo "$(date +%T) hard set: astar + $controller"
  python3 eval/run_batch.py --seeds $HARD --obstacles 20 --planner astar --controller $controller \
    --jobs $J --out $OUT/trials_hard
done

# 3. mapping: to the goal and back with slam_toolbox running too
echo "$(date +%T) mapping"
for seed in $MAPPING; do
  [ -f $OUT/mapping/$(printf %04d $seed)_astar_mpc_home_slam.json ] || \
    python3 eval/run_trial.py --seed $seed --return-home --slam --out $OUT/mapping
done
python3 eval/compare_maps.py "$OUT/mapping/*_slam.json" | tee $OUT/mapping_runs.txt
tail -1 $OUT/mapping_runs.txt | sed 's/^SUMMARY //' > $OUT/mapping.json

# 4. the offline planner benchmark
echo "$(date +%T) planner benchmark"
python3 eval/make_bench_maps.py --seeds $MAIN --out $OUT/bench_maps
ros2 run nav_stack bench_planners $OUT/bench_maps/cases.txt 5 > $OUT/bench.csv

echo "$(date +%T) ALL DONE"