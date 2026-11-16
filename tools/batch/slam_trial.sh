#!/bin/bash
# Short SLAM tuning run: slam_trial.sh <label> <seed> <lidar> <texture> <seconds> [overrides]
# Prints the pose-error lines for the run. Overrides: "Key=value,Key2=value2" (RTAB-Map SLAM).
LABEL=$1; SEED=$2; LIDAR=$3; TEX=$4; SECS=$5
export AIRMOUSE_SLAM_OVERRIDES="$6"
GT=false bash /ws/tools/batch/mission_run.sh $SEED $LIDAR $TEX 1.0 $SECS > /dev/null 2>&1
{ echo "=== $LABEL (seed $SEED lidar $LIDAR $TEX, ${SECS}s) [$6]"; grep -E "POSE|ODOMETRY|MIN CLEAR|STATE (DONE|ABORT)|TIMEOUT|^seed" /tmp/mission_${SEED}_${LIDAR}.log; grep -a -c "localization" /tmp/mission_launch.log; } >> /tmp/slam_trials.log
