#!/bin/bash
# grid_sim over many seeds in parallel: grid_sim_batch.sh <first> <last> <lidar> [jobs]
# Output: /tmp/gridsim_<lidar>.log (one line per seed + nothing else), sorted at the end.
A=$1; B=$2; L=$3; J=${4:-4}
OUT=/tmp/gridsim_$L.log; : > $OUT
seq $A $B | xargs -P $J -I{} sh -c "cd /ws && python3 tools/grid_sim.py --seeds {} --lidar $L | grep '^seed' >> $OUT"
sort -n -k2 $OUT -o $OUT
echo "GRIDSIM DONE $(grep -c ' OK ' $OUT)/$(wc -l < $OUT) OK" >> $OUT
