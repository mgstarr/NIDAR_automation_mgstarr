# Phase 2 batch: 6 mapping runs, seeds 1-6, LiDAR on/off, plain/textured. Results in /tmp/batch.log.
bash /ws/tools/batch/kill_sim.sh
for run in "1 true plain" "2 false textured" "3 true textured" "4 false textured" "5 true plain" "6 false plain"; do
  bash /ws/tools/batch/map_run.sh $run 2.0 >> /tmp/batch.log 2>&1
done
echo BATCH_DONE >> /tmp/batch.log
