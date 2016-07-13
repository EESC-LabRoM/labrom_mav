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

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
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

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS actionlib_msgs labrom_control #geometry_msgs message_generation roscpp rospy sensor_msgs std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
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

## Declare a C++ executable
add_executable(blind_node nodes/blind_node.cpp)
target_link_libraries(blind_node ${catkin_LIBRARIES} ${PROJECT_NAME} )
add_dependencies(blind_node ${blind_action_EXPORTED_TARGETS})

