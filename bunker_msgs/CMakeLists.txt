cmake_minimum_required(VERSION 3.0.2)
project(bunker_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  BunkerStatus.msg
  BunkerMotorState.msg
  BunkerDriverState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bunker_python
#  CATKIN_DEPENDS rospy bunker_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)
