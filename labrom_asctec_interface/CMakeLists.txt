cmake_minimum_required(VERSION 2.8.3)
project(labrom_asctec_commands)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
)

## Declare a C++ executable
add_executable(node src/node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(node
  ${catkin_LIBRARIES}
)

