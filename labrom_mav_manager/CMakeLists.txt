cmake_minimum_required(VERSION 2.8.3)
project(labrom_mav_manager)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  eigen_conversions
  labrom_mav_common
  labrom_mav_control  
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(Eigen3 REQUIRED)
catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}  
)

add_executable(state_machine_node src/state_machine.cpp)
target_link_libraries(state_machine_node  ${catkin_LIBRARIES})
