#!/bin/bash
# Phase 4 automated goal test on one maze (headless): goal_run.sh <seed> [goals] [texture] [lidar]
# Output: /tmp/goal_run_<seed>.log (driver lines + MIN CLEARANCE line).
SEED=${1:-1}; GOALS=${2:-7}; TEX=${3:-plain}; LIDAR=${4:-true}
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash; cd /ws
export PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0
LOG=/tmp/goal_run_${SEED}.log; : > $LOG
bash tools/batch/kill_sim.sh >/dev/null 2>&1
ros2 launch airmouse_bringup sim.launch.py seed:=$SEED wall_texture:=$TEX use_lidar:=$LIDAR \
    use_ground_truth:=true gui:=false rviz:=false > /tmp/goal_sim.log 2>&1 &
sleep 15
ros2 launch airmouse_bringup localization.launch.py use_ground_truth:=true use_lidar:=$LIDAR > /tmp/goal_loc.log 2>&1 &
ros2 launch airmouse_bringup navigation.launch.py use_sim_time:=true seed:=$SEED wall_texture:=$TEX > /tmp/goal_nav.log 2>&1 &
sleep 8
[ -n "$BAG" ] && { rm -rf /tmp/goal_bag_$SEED; ros2 bag record -o /tmp/goal_bag_$SEED /cmd_vel /nav/cmd_vel_raw /ground_truth/odom /nav/controller_state /nav/path > /dev/null 2>&1 & }
python3 tools/clearance.py --layout src/airmouse_description/worlds/generated/seed_${SEED}_${TEX} >> $LOG 2>&1 &
CL=$!
python3 tools/batch/goal_driver.py --goals $GOALS --seed $SEED >> $LOG 2>&1
kill -INT $CL; sleep 2
bash tools/batch/kill_sim.sh >/dev/null 2>&1
echo "DONE seed $SEED" >> $LOG
