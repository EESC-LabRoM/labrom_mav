cmake_minimum_required(VERSION 2.8.3)
project(blind_action)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  labrom_control
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
add_action_files(
    DIRECTORY action
    FILES TakeOff.action Climb.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib_msgs labrom_control #geometry_msgs message_generation roscpp rospy sensor_msgs std_msgs
  LIBRARIES  ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} 
   src/take_off_server.cpp
   src/take_off_client.cpp 
   src/climb_client.cpp 
   src/climb_server.cpp 
   src/blind.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

