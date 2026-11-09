# Stop every sim/mapping process (bracket patterns so pkill never matches this script).
pkill -INT -f "[r]os2 launch"; sleep 3
pkill -9 -f "[g]z sim|[p]arameter_bridge|[s]tatic_transform|[s]im_bridge_node|[p]oint_cloud_xyz|[r]tabmap|[s]urvey_flight|[t]akeoff.py|[r]viz2|[g]oal_planner_node|[c]ontroller_node|[s]afety_hook_node|[c]learance.py|[g]oal_driver|[f]ly_path|[r]os2 bag"
sleep 1; true
