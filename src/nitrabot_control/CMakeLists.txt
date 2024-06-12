cmake_minimum_required(VERSION 3.0.2)
project(nitrabot_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
  robot_state_publisher rviz std_msgs genmsg urdf xacro tf message_generation nav_msgs 
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

Set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pure_pursuit.cpp
    )

set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/pure_pursuit.h
    )

add_library(control ${SOURCES} ${HEADERS})

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node ${catkin_LIBRARIES} control)

add_executable(feedforward src/feedforward.cpp)
