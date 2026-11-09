#!/bin/bash
# One headless Gazebo mission: mission_run.sh <seed> <lidar> [texture] [rtf] [timeout_s]
# Logs: /tmp/mission_<seed>_<lidar>.log (state changes, clearance), bag /tmp/mission_bag_<seed>_<lidar>
SEED=${1:-1}; LIDAR=${2:-true}; TEX=${3:-plain}; RTF=${4:-1.0}; TMO=${5:-2400}
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash; cd /ws
export PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0
LOG=/tmp/mission_${SEED}_${LIDAR}.log; : > $LOG
bash tools/batch/kill_sim.sh >/dev/null 2>&1
ros2 launch airmouse_bringup mission.launch.py seed:=$SEED use_lidar:=$LIDAR wall_texture:=$TEX \
    use_ground_truth:=true gui:=false rviz:=false real_time_factor:=$RTF > /tmp/mission_launch.log 2>&1 &
sleep 25
rm -rf /tmp/mission_bag_${SEED}_${LIDAR}
ros2 bag record -o /tmp/mission_bag_${SEED}_${LIDAR} /ground_truth/odom /mission/state /nav/path > /dev/null 2>&1 &
python3 tools/clearance.py --layout src/airmouse_description/worlds/generated/seed_${SEED}_${TEX} >> $LOG 2>&1 &
CL=$!
ros2 service call /mission/start std_srvs/srv/Trigger >> $LOG 2>&1
T0=$(date +%s)
# Optional mid-mission service test: CALL_SRV=abort|return_home CALL_AFTER=<wall seconds>
if [ -n "$CALL_SRV" ]; then
  ( sleep ${CALL_AFTER:-120}; echo "$(( $(date +%s) - T0 ))s CALL /mission/$CALL_SRV" >> $LOG;
    ros2 service call /mission/$CALL_SRV std_srvs/srv/Trigger | grep -a response -A1 >> $LOG ) &
fi
LAST=""
while true; do
  S=$(timeout 5 ros2 topic echo --once /mission/state --field data 2>/dev/null | head -1)
  [ -n "$S" ] && [ "$S" != "$LAST" ] && { echo "$(( $(date +%s) - T0 ))s STATE $S" >> $LOG; LAST=$S; }
  [ "$S" = "DONE" ] && break
  if [ "$S" = "ABORT" ]; then sleep 25; break; fi
  [ $(( $(date +%s) - T0 )) -gt $TMO ] && { echo "TIMEOUT" >> $LOG; break; }
  sleep 5
done
sleep 3
kill -INT $CL; sleep 2
pkill -INT -f "[r]os2 bag record"; sleep 2
grep -a -E "mission:|exploration complete|return budget|no path home|geofence|localization lost|waiting|blacklisted" /tmp/mission_launch.log | tail -25 >> $LOG
cp /tmp/mission_launch.log /tmp/mission_launch_${SEED}_${LIDAR}.log
bash tools/batch/kill_sim.sh >/dev/null 2>&1
ros2 bag reindex /tmp/mission_bag_${SEED}_${LIDAR} -s mcap > /dev/null 2>&1
python3 tools/mission_report.py --bag /tmp/mission_bag_${SEED}_${LIDAR} --seed $SEED 2>/dev/null | grep '^seed' >> $LOG
echo "DONE seed $SEED lidar $LIDAR" >> $LOG
