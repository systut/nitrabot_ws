FROM osrf/ros:noetic-desktop-full

# copy install scripts
COPY ./nitrabot_dockerfiles/install /root/install

# copy directory specific to scart
COPY ./ /root/catkin_ws/src/

# update and install dependencies
RUN /bin/sh -e -c /root/install/i_dep.sh

# set environment variable for ros
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc

# install ros dependencies
WORKDIR /root/catkin_ws 

# RUN rosdep update && \
#     rosdep install --from-paths src --ignore-src -r -y

# RUN catkin config --skiplist nitrabot_communication

# Build ROS packages
# RUN catkin build nitrabot_control && \
#    chmod 777 /root/catkin_ws/devel/setup.bash && \
#    chmod 755 /root