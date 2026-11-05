# One mapping run: sim + RTAB-Map headless, survey flight, save map, score. Args: seed lidar texture rtf
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash; cd /ws; export PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0
S=$1; L=$2; T=$3; R=${4:-1.0}; NAME=seed${S}_$([ $L = true ] && echo lidar || echo nolidar)_$T
ros2 launch airmouse_bringup sim.launch.py seed:=$S use_lidar:=$L wall_texture:=$T use_ground_truth:=true gui:=false rviz:=false real_time_factor:=$R > /tmp/sim_$NAME.log 2>&1 &
sleep 20
ros2 launch airmouse_bringup localization.launch.py use_lidar:=$L use_ground_truth:=true > /tmp/loc_$NAME.log 2>&1 &
sleep 10
( sleep 60; timeout 15 ros2 topic hz /map --window 10 2>/dev/null | grep "average rate" | tail -1 > /tmp/hz_$NAME.txt ) &
START=$(date +%s)
timeout 3000 python3 tools/survey_flight.py --layout src/airmouse_description/worlds/generated/seed_${S}_$T > /tmp/survey_$NAME.log 2>&1
echo "$NAME: survey $(tail -1 /tmp/survey_$NAME.log | grep -o 'survey done' || echo FAILED) in $(( $(date +%s) - START )) s; map rate: $(cat /tmp/hz_$NAME.txt)"
sleep 3
python3 tools/save_map.py --name $NAME --seed $S --use-lidar $L --wall-texture $T > /dev/null
python3 tools/score_map.py --map src/airmouse_nav/test/data/maps/$NAME.yaml --layout src/airmouse_description/worlds/generated/seed_${S}_$T/layout.yaml | tr -d '\n '; echo
bash /ws/tools/batch/kill_sim.sh
