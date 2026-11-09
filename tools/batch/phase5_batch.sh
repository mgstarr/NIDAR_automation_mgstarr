#!/bin/bash
# Phase 5 exit runs (headless, hours): grid_sim 50 seeds x LiDAR on/off, 10 Gazebo missions
# (seeds 11-15 with LiDAR, 16-20 camera only, alternating textures, RTF 2), abort and recall tests.
# Summary: /tmp/phase5.log
OUT=/tmp/phase5.log; : > $OUT
bash /ws/tools/batch/grid_sim_batch.sh 1 50 true 12;  tail -1 /tmp/gridsim_true.log  >> $OUT
bash /ws/tools/batch/grid_sim_batch.sh 1 50 false 12; tail -1 /tmp/gridsim_false.log >> $OUT
for run in "11 true plain" "12 true textured" "13 true plain" "14 true textured" "15 true plain" \
           "16 false textured" "17 false plain" "18 false textured" "19 false plain" "20 false textured"; do
  set -- $run
  bash /ws/tools/batch/mission_run.sh $1 $2 $3 2.0 2400
  { echo "=== mission seed $1 lidar $2 $3"; grep -E "STATE|MIN CLEAR|^seed|TIMEOUT|exploration complete" /tmp/mission_$1_$2.log | grep -v lost; } >> $OUT
done
CALL_SRV=return_home CALL_AFTER=120 bash /ws/tools/batch/mission_run.sh 21 true plain 2.0 900
{ echo "=== recall test seed 21"; grep -E "STATE|CALL|success|MIN CLEAR" /tmp/mission_21_true.log | grep -v lost; } >> $OUT
CALL_SRV=abort CALL_AFTER=120 bash /ws/tools/batch/mission_run.sh 22 false plain 2.0 900
{ echo "=== abort test seed 22"; grep -E "STATE|CALL|success|MIN CLEAR" /tmp/mission_22_false.log | grep -v lost; } >> $OUT
echo "PHASE5 BATCH DONE" >> $OUT
