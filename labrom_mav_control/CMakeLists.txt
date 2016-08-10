cmake_minimum_required(VERSION 2.8.3)
project(labrom_mav_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  labrom_control
  nav_msgs
  roscpp
  std_msgs
  tf
  trajectory_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#executables
add_executable(velocity_linear_node nodes/velocity_linear.cpp)
target_link_libraries(velocity_linear_node ${catkin_LIBRARIES})