FROM ros:melodic-ros-core

# copy install scripts
COPY ./agribot_dockerfiles/install /root/install

# copy directory specific to scart
COPY ./nitrabot_control /root/catkin_ws/src/nitrabot_control

# update and install dependencies
RUN /bin/sh -e -c /root/install/i_dep.melodic.sh

# set environment variable for ros
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc
RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc

# install ros dependencies
WORKDIR /root/catkin_ws 
RUN catkin config --extend /opt/ros/melodic
# RUN rosdep update && \
#     rosdep install --from-paths src --ignore-src -r -y

# Build ROS packages
RUN catkin build nitrabot_control && \
   chmod 777 /root/catkin_ws/devel/setup.bash && \
   chmod 755 /root