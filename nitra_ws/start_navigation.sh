bash start_lidar.sh
sudo chmod 666 /dev/ttyACM0
source ~/nitra_ws/devel/setup.bash
roslaunch nitra_robot navigation.launch
