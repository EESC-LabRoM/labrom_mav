cmake_minimum_required(VERSION 2.8.3)
project(labrom_mav_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  labrom_mav_common
  roscpp
  rospy
)
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
INCLUDE_DIRS include
LIBRARIES labrom_mav_control
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}  
)

add_library(labrom_mav_control
src/PoseLinear.cpp)

target_link_libraries(labrom_mav_control ${catkin_LIBRARIES})
