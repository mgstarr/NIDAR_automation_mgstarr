#!/bin/bash
# Phase 4 exit test: 7 automated goals on each of 3 mazes. Summary in /tmp/goal_batch.log.
: > /tmp/goal_batch.log
for run in "1 plain true" "3 textured true" "5 plain false"; do
  set -- $run
  BAG=1 bash /ws/tools/batch/goal_run.sh $1 7 $2 $3
  { echo "=== seed $1 $2 lidar=$3"; grep -v "so far" /tmp/goal_run_$1.log; } >> /tmp/goal_batch.log
done
echo BATCH DONE >> /tmp/goal_batch.log
