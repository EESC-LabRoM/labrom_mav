cmake_minimum_required(VERSION 2.8.3)
project(labrom_mav_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  actionlib_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
add_action_files(
    DIRECTORY action
    FILES TakeOff.action Landing.action 
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
  CATKIN_DEPENDS actionlib_msgs
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} 
   src/take_off_client.cpp
   src/landing_client.cpp  
   src/blind.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(take_off_node src/take_off_server.cpp)
target_link_libraries(take_off_node  ${catkin_LIBRARIES})
add_dependencies(take_off_node ${labrom_mav_manager_EXPORTED_TARGETS})

add_executable(landing_node src/landing_server.cpp)
target_link_libraries(landing_node  ${catkin_LIBRARIES})
add_dependencies(landing_node ${labrom_mav_manager_EXPORTED_TARGETS})

# Executables
add_executable(manager_node src/manager.cpp)
target_link_libraries(manager_node ${catkin_LIBRARIES} ${PROJECT_NAME})
