bash start_lidar.sh
sudo chmod 666 /dev/ttyUSB1
source ~/nitra_ws/devel/setup.bash
roslaunch nitra_robot bringup.launch
