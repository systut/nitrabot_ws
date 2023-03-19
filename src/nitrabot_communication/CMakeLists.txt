cmake_minimum_required(VERSION 3.8)
project(nitrabot_communication)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs std_msgs tf message_generation
)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_communication/robot_data.cpp
    )

set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_communication/robot_data.h
)

add_library(industrial ${SOURCES} ${HEADERS})


catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RobotWheelVel.msg
  EncoderWheelVel.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs std_msgs tf message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/bumper_control_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(robot_communication src/robot_communication/robot_communication.cpp)
target_link_libraries(robot_communication ${catkin_LIBRARIES} serial industrial)  

add_executable(encoder_to_odom src/robot_communication/encoder_to_odom.cpp)
target_link_libraries(encoder_to_odom ${catkin_LIBRARIES} serial industrial) 

add_executable(command_to_odom src/robot_communication/command_to_odom.cpp)
target_link_libraries(command_to_odom ${catkin_LIBRARIES} serial industrial) 