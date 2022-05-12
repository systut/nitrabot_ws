echo "Create map at current time"
source ~/cartographer_ws/devel_isolated/setup.bash
# rosrun map_server map_saver -f $(rospack find "lumi_robot")/maps/mymap
# Finish the first trajectory. No further data will be accepted on it.
rosservice call /finish_trajectory 0

# Ask Cartographer to serialize its current state.
# (press tab to quickly expand the parameter syntax)
rosservice call /write_state "{filename: '${HOME}/Lumi-Robotics/src/lumi_robot/maps/map.pbstream', include_unfinished_submaps: "true"}"

