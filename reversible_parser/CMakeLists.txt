cmake_minimum_required(VERSION 2.8.3)
project(reversible_parser)

message(WARNING "Forcing (empty) build type for package abc_demo!")
set(CMAKE_BUILD_TYPE Debug)


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

################################################
## Find libs 		                              ##
################################################

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  RobWork
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
#  LIBRARIES d7cp_proxys reversible_dsl
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  include/reversible_parser
  src
  ${catkin_INCLUDE_DIRS}
  ${ROBWORK_INCLUDE_DIRS}
  ${ROBWORKSTUDIO_INCLUDE_DIRS}
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
add_library(reversible_parser
	src/ast/AST2Graphviz.cpp
	src/ast/ASTNode.cpp
	src/ast/ASTProcessor.cpp
	src/code_genneration/CodeGen.cpp
	src/lexer/RegexLexer.cpp
	src/lexer/Token.cpp
	src/lexer/TokenBuffer.cpp
	src/lexer/TokenType.cpp
	src/parser/Combinator.cpp
	src/parser/Error23.cpp
	src/parser/ListCombinator.cpp
	src/parser/OrCombinator.cpp
	src/parser/ParseResult.cpp
	src/parser/SemanticProcessor.cpp
	src/parser/SequenceCombinator.cpp
	src/parser/Terminal.cpp
	src/support/File23.cpp
	src/support/GraphvizPrinter.cpp
	src/D7pcAstRefinement.cpp
	src/D7pcAstType.cpp
	src/D7pcParser.cpp
	src/D7pcTokenType.cpp
)

## Specify libraries to link a library against
target_link_libraries(reversible_parser
	${catkin_LIBRARIES}
	${ROBWORK_LIBRARIES}
	${Boost_LIBRARIES}
)


## Declare a cpp executable
add_executable(reversible_parser_node	src/main.cpp)

## Add cmake target dependencies of the executable
add_dependencies(reversible_parser_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(reversible_parser_node	reversible_parser )

#############
## Install ##
#############

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test 
	test/test_reversible_parser.cpp
	src/TestTools.cpp 
)
if(TARGET ${PROJECT_NAME}-test)
	target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

