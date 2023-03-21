#!/bin/sh
set -e
basic_dep="git \
           curl \
           nano \
           vim \
           python3-catkin-tools \
           python3-pip \
		   libserial-dev
	  "

ros_dep="ros-melodic-tf \
         ros-melodic-robot-state-publisher \
         ros-melodic-rviz \
         ros-melodic-xacro"
          
apt-get update
apt-get upgrade -y
apt-get install -y $basic_dep
DEBIAN_FRONTEND=noninteractive apt-get install -y $ros_dep
apt-get autoremove -y
apt-get clean -y