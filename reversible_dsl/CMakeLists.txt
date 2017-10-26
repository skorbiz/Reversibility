cmake_minimum_required(VERSION 2.8.3)
project(reversible_dsl)


################################################
## Robwork	 		                          ##
################################################

SET(RW_ROOT $ENV{RW_ROOT})
SET(RWSTUDIO_ROOT $ENV{RWS_ROOT})
SET(RWHW_ROOT $ENV{RWHW_ROOT})

SET(RobWork_DIR ${RW_ROOT}/cmake) 
SET(RobWorkStudio_DIR $ENV{RWS_ROOT}/cmake)
SET(RobWorkHardware_DIR $ENV{RWHW_ROOT}/cmake)

MESSAGE("\n \n ROBWORK_LIBRARY_DIRS:\n ${ROBWORK_LIBRARY_DIRS} \n ${ROBWORKSIM_LIBRARY_DIRS} ")

SET(RobWork_DIR ${RW_ROOT}/cmake) 

#################################################
### Trajopt and supporting libs                ##
#################################################
#
# Find boost
#FIND_PACKAGE(Boost REQUIRED thread system program_options) 
#
# Find OpenRAVE (Needed by TrajOpt)
#FIND_PACKAGE(OpenRAVE REQUIRED)
#SET(OpenRAVE_BOTH_LIBRARIES ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
#
# Find OMPL
#FIND_PACKAGE(OMPL REQUIRED)
#
# Find TrajOpt and its external libraries 
#set(TrajOptExtDir $ENV{TRAJOPT_LIB_DIR})
#set(TrajOptRoot $ENV{TRAJOPT_ROOT})
#
#MESSAGE(${TrajOptRoot})
#MESSAGE(${TrajOptExtDir})
#
#FIND_LIBRARY(scolib sco ${TrajOptExtDir} REQUIRED)
#FIND_LIBRARY(jsonlib json ${TrajOptExtDir} REQUIRED)
#FIND_LIBRARY(osgviewerlib osgviewer ${TrajOptExtDir} REQUIRED)
#FIND_LIBRARY(TrajoptLibrary trajopt ${TrajOptExtDir} REQUIRED)
#
#SET(jsonIncDir ${TrajOptRoot}/ext/json/include)
#SET(TrajOptIncDir ${TrajOptRoot}/src)
#SET(TrajOptTestIncDir ${TrajOptRoot}/src/trajopt/test)
#

################################################
## Find libs 		                              ##
################################################

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  RobWork
  RobWorkStudio
  d7cp_proxys
#  d7pc_pathplanner
)

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
  INCLUDE_DIRS include src
  LIBRARIES d7cp_proxys
  CATKIN_DEPENDS roscpp
  	d7cp_proxys
#  DEPENDS system_lib
)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${ROBWORK_INCLUDE_DIRS}
  ${ROBWORKSTUDIO_INCLUDE_DIRS}
  ${d7cp_proxys_INCLUDE_DIRS}  
  ${BOOST_INCLUDE_DIRS}
)

link_directories( 
	${ROBWORK_LIBRARY_DIRS} 
	${ROBWORKSTUDIO_LIBRARY_DIRS}
	${ROBWORKHARDWARE_LIBRARY_DIRS}
) 	# A better way has to be found later



###########
## Build ##
###########


## Declare a C++ library
add_library(reversible_dsl
	src/preplanner/OnlinePlanners.cpp
	src/preplanner/Path.cpp
	src/preplanner/Preplanner.cpp
	src/skill/AnyfeederPickSkill.cpp
	src/skill/spiral_skill.cpp
	src/skill/wiggle_skill.cpp
	src/skill/Compliant_pih_skill.cpp
	src/colors.cpp
	src/Converter.cpp
	src/D7cpWorkcell.cpp
	src/DynamicGrasp.cpp
  	src/KukaIIWAPlanner.cpp
  	src/Move.cpp
  	src/MoveLin.cpp
  	src/Move2Q.cpp
	src/Planner.cpp
)

## Specify libraries to link a library against
target_link_libraries(reversible_dsl
# my_dsl_common_library
	${catkin_LIBRARIES}
	${ROBWORK_LIBRARIES}
	${Boost_LIBRARIES}
)


## Declare a cpp executable
add_executable(reversible_dsl_node	src/main.cpp  )

## Add cmake target dependencies of the executable
add_dependencies(reversible_dsl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(reversible_dsl_node	reversible_dsl )


## FULL ASSEMBLY SEQUENCE
add_executable(full_sequence	src/main_full_sequence.cpp  )
add_dependencies(full_sequence ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(full_sequence	reversible_dsl )

#############
## Install ##
#############

#############
## Testing ##
#############
