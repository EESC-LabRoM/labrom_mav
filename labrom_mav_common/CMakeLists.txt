cmake_minimum_required(VERSION 2.8.3)
project(labrom_mav_common)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  nav_msgs
)


find_package(OpenCV REQUIRED)


find_package(Eigen3 REQUIRED)


add_message_files(
  FILES
  MavCmd.msg
  TeleOpCmd.msg   
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime 
INCLUDE_DIRS include
LIBRARIES labrom_mav_common
)


###########
## Build ##
###########


# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}  
  ${EIGEN3_INCLUDE_DIR}  
)

add_library(labrom_mav_common
  src/conversions.cpp
)

target_link_libraries(labrom_mav_common ${catkin_LIBRARIES} ${OpenCV_LIBS})

### INSTALL ####
install(TARGETS labrom_mav_common
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
        
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
