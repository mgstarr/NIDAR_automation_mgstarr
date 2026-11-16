#!/bin/bash
# Phase 5 + Phase 6 exit runs in Gazebo (headless; about 9 hours). Summary: /tmp/phase56.log
#   Phase 5 (ground truth, RTF 2): 10 mazes (seeds 11-15 LiDAR, 16-20 camera only), recall, abort.
#   Phase 6 (own localization, RTF 1): 10 LiDAR missions (seeds 21-30, plain/textured),
#     3 camera-only on textured walls, 2 camera-only on plain walls, sensor-drop failsafe test.
OUT=/tmp/phase56.log; : > $OUT
KEEP="STATE|MIN CLEAR|^seed|TIMEOUT|POSE|ODOMETRY|MAP SCORE|CALL|DROP|success"
one() {  # one <label> <seed> <lidar> <texture> <rtf> <timeout>
  bash /ws/tools/batch/mission_run.sh $2 $3 $4 $5 $6
  { echo "=== $1 seed $2 lidar $3 $4"; grep -E "$KEEP" /tmp/mission_$2_$3.log | grep -v "lost!\|WARNING";
    grep -a -E "localization|mission: " /tmp/mission_launch.log | tail -8 | cut -c1-160; } >> $OUT
}
export GT=true
for run in "11 true plain" "12 true textured" "13 true plain" "14 true textured" "15 true plain" \
           "16 false textured" "17 false plain" "18 false textured" "19 false plain" "20 false textured"; do
  set -- $run; one "P5 ground-truth" $1 $2 $3 2.0 2400
done
CALL_SRV=return_home CALL_AFTER=120 one "P5 recall test" 31 true plain 2.0 900
CALL_SRV=abort CALL_AFTER=120 one "P5 abort test" 32 false plain 2.0 900
export GT=false
for run in "21 true plain" "22 true textured" "23 true plain" "24 true textured" "25 true plain" \
           "26 true textured" "27 true plain" "28 true textured" "29 true plain" "30 true textured"; do
  set -- $run; one "P6 SLAM lidar" $1 $2 $3 1.0 2700
done
for run in "33 false textured" "34 false textured" "35 false textured" "36 false plain" "37 false plain"; do
  set -- $run; one "P6 SLAM camera-only" $1 $2 $3 1.0 2700
done
DROP_AFTER=150 one "P6 sensor-drop test" 38 true plain 1.0 600
echo "PHASE56 BATCH DONE" >> $OUT
