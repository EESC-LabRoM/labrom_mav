cmake_minimum_required(VERSION 2.8.3)
project(labrom_mav_interface)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  labrom_mav_common
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ executable
add_executable(vrep_node nodes/vrep_interface.cpp)
target_link_libraries(vrep_node ${catkin_LIBRARIES})

add_executable(crazyflie_node nodes/crazyflie_interface.cpp)
target_link_libraries(crazyflie_node ${catkin_LIBRARIES})

add_executable(joy_node nodes/joy_interface.cpp)
target_link_libraries(joy_node ${catkin_LIBRARIES})

add_executable(vrpn_node nodes/vrpn_interface.cpp)
target_link_libraries(vrpn_node ${catkin_LIBRARIES})

add_executable(print_node nodes/print.cpp)
target_link_libraries(print_node ${catkin_LIBRARIES})

add_executable(pelican_node nodes/pelican_interface.cpp)
target_link_libraries(pelican_node ${catkin_LIBRARIES})
