cmake_minimum_required(VERSION 2.8.3)
project(labrom_asctec_interface)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  asctec_hl_comm 
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(mav_ctrl_interface_node nodes/mav_ctrl.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(mav_ctrl_interface_node ${catkin_LIBRARIES})

