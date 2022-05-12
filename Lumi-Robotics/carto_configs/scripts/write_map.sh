# Finish the first trajectory. No further data will be accepted on it.
rosservice call /finish_trajectory 0

# Ask Cartographer to serialize its current state.
# (press tab to quickly expand the parameter syntax)
rosservice call /write_state "{filename: '${HOME}/Lumi-Robotics/carto_configs/maps/map.pbstream', include_unfinished_submaps: "true"}"

# Delete previous map
rm -rf mymap.*

# Convert to yaml
rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/Lumi-Robotics/carto_configs/maps/mymap -pbstream_filename=${HOME}/Lumi-Robotics/carto_configs/maps/map.pbstream

