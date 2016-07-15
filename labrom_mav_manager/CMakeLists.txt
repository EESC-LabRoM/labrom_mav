cmake_minimum_required(VERSION 2.8.3)
project(labrom_mav_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  blind_action
  labrom_control
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system)
###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS blind_action labrom_control roscpp
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
   src/manager.cpp
)

# Executables
add_executable(manager_node nodes/manager_node.cpp)
target_link_libraries(manager_node ${catkin_LIBRARIES} ${PROJECT_NAME})
