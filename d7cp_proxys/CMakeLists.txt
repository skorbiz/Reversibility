cmake_minimum_required(VERSION 2.8.3)
project(d7cp_proxys)


SET(RW_ROOT $ENV{RW_ROOT})
SET(RobWork_DIR ${RW_ROOT}/cmake) 

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
	RobWork
	anyfeeder_driver
	wsg_50_common
	iiwa_msgs
	d7pc_msgs
	robot_movement_interface
	iiwa_driver
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## C++11/14 support                           ##
################################################

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++1y" COMPILER_SUPPORTS_CXX1y)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX14)
    add_definitions(-std=c++14)    #Uses the 2011 version of C++
    message("Compiled with c++14")
elseif(COMPILER_SUPPORTS_CXX1y)
    add_definitions(-std=c++1y)    #Uses the 2011 version of C++
    message("Compiled with c++1y")
elseif(COMPILER_SUPPORTS_CXX0X)
 	add_definitions(-std=c++11)    #Uses the 2011 version of C++
    message("Compiled with c++11")
else()
    message(FATAL_ERROR: "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES d7cp_proxys
  CATKIN_DEPENDS roscpp
	roslib
	anyfeeder_driver
  wsg_50_common
  iiwa_msgs
  d7pc_msgs
  robot_movement_interface
  iiwa_driver
#  DEPENDS system_lib
)

include_directories(
  include/d7cp_proxys
  ${catkin_INCLUDE_DIRS}
  ${ROBWORK_INCLUDE_DIRS}
)

link_directories( 
  ${ROBWORK_LIBRARY_DIRS} 
) # A better way has to be found later


###########
## Build ##
###########

# Declare a C++ library
add_library(d7cp_proxys
#  src/iiwa/iiwa_robot.cpp
#  src/iiwa/RobworkInterface.cpp
#  src/iiwa/RosProxy.cpp
  src/gripper/D7PCGripper.cpp
  src/gripper/WSG25Gripper.cpp
  src/gripper/WSG25Proxy.cpp
  src/robot_movement_interface/RobworkInterface.cpp
  src/robot_movement_interface/RosProxy.cpp
  src/InspectionSystem.cpp
  src/VisionSystem.cpp
  src/Anyfeeder.cpp
)

# Specify libraries to link a library against
target_link_libraries(d7cp_proxys
  ${catkin_LIBRARIES}
  ${ROBWORK_LIBRARIES}
)


#############
## Install ##
#############

#############
## Testing ##
#############
