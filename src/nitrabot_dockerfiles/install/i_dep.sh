#!/bin/sh
set -e
basic_dep="git \
           curl \
           nano \
           vim \
           python3-catkin-tools \
           python3-pip \
	  "

ros_dep="ros-noetic-tf \
         ros-noetic-robot-state-publisher \
         ros-noetic-rviz \
         ros-noetic-roslint \
         ros-noetic-nmea-navsat-driver \
         ros-noetic-xacro \
         ros-noetic-ros-control \
         ros-noetic-ros-controllers"

python_dep="rospy-message-converter \
         socketio"

apt-get update
apt-get upgrade -y
apt-get install -y $basic_dep
DEBIAN_FRONTEND=noninteractive apt-get install -y $ros_dep
apt-get autoremove -y
apt-get clean -y
python3 -m pip install $python_dep