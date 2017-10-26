cmake_minimum_required(VERSION 2.8.3)
project(reversible_dsl_v1)

SET(RW_ROOT $ENV{RW_ROOT})
SET(RWSTUDIO_ROOT $ENV{RWS_ROOT})
SET(RWHW_ROOT $ENV{RWHW_ROOT})

SET(RobWork_DIR ${RW_ROOT}/cmake) 
SET(RobWorkStudio_DIR $ENV{RWS_ROOT}/cmake)
SET(RobWorkHardware_DIR $ENV{RWHW_ROOT}/cmake)

MESSAGE("\n \n ROBWORK_LIBRARY_DIRS:\n ${ROBWORK_LIBRARY_DIRS} \n ${ROBWORKSIM_LIBRARY_DIRS} ")

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    add_definitions(-std=c++11)    #Uses the 2011 version of C++
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    add_definitions(-std=c++0x)    #Uses the 2011 version of C++ (ubuntu 12.04)
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()



find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  rospy
  std_msgs
  actionlib
  carmen_hw_interface
  dsl_common_v1
  RobWork
  RobWorkStudio
  RobWorkHardware
)

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
  CATKIN_DEPENDS carmen_hw_interface
  CATKIN_DEPENDS dsl_common_v1
  CATKIN_DEPENDS message_runtime 
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dsl_common_INCLUDE_DIRS}
  ${ROBWORK_INCLUDE_DIRS}
  ${ROBWORKSTUDIO_INCLUDE_DIRS}
  ${ROBWORKHARDWARE_INCLUDE_DIRS}
)

LINK_DIRECTORIES( 
	${ROBWORK_LIBRARY_DIRS} 
	${ROBWORKSTUDIO_LIBRARY_DIRS}
	${ROBWORKHARDWARE_LIBRARY_DIRS}
	) # A better way has to be found later


add_library(
rdsllib_v1
    src/language/AssemblyProgram.cpp
    src/language/FMbuilder.cpp
    src/language/SequenceBuilder.cpp
    src/language/xml/XMLloader.cpp
    src/model/basemodel/argument/quantities/ForceModeArgument.cpp
    src/model/basemodel/argument/quantities/IOPorts.cpp
    src/model/basemodel/argument/quantities/JointConfiguration.cpp
    src/model/basemodel/argument/quantities/Switch.cpp
    src/model/basemodel/argument/Argument.cpp    
    src/model/basemodel/argument/Datatype.cpp
    src/model/basemodel/BaseProgram.cpp
    src/model/basemodel/command/Command.cpp
    src/model/basemodel/command/CommandExecutable.cpp
    src/model/basemodel/command/derivedCommands/Action.cpp
    src/model/basemodel/command/derivedCommands/ActionFactory.cpp
    src/model/basemodel/command/derivedCommands/ActionIKSolver.cpp
    src/model/basemodel/command/derivedCommands/EmptyLinker.cpp
    src/model/basemodel/command/derivedCommands/ForceMode.cpp
    src/model/basemodel/command/derivedCommands/Grasp.cpp
    src/model/basemodel/command/derivedCommands/intelligentMove/ForceStatistics.cpp
    src/model/basemodel/command/derivedCommands/intelligentMove/ForceStatisticsSingle.cpp
    src/model/basemodel/command/derivedCommands/intelligentMove/IntelligentActionModelInterface.cpp
    src/model/basemodel/command/derivedCommands/intelligentMove/IntelligentMove.cpp
    src/model/basemodel/command/derivedCommands/intelligentMove/IntelligentMoveModelInterface.cpp
    src/model/basemodel/command/derivedCommands/intelligentMove/IntelligentTrajectory.cpp
    src/model/basemodel/command/derivedCommands/intelligentMove/ManualExecutionSelector.cpp
    src/model/basemodel/command/derivedCommands/intelligentMove/MinimumTresholdCalculator.cpp
    src/model/basemodel/command/derivedCommands/intelligentMove/Record.cpp
    src/model/basemodel/command/derivedCommands/intelligentMove/RecordDataset.cpp
    src/model/basemodel/command/derivedCommands/intelligentMove/RecordPrintUtillities.cpp
    src/model/basemodel/command/derivedCommands/intelligentMove/Statistics.cpp
    src/model/basemodel/command/derivedCommands/intelligentMove/XMLRecordLoader.cpp
    src/model/basemodel/command/derivedCommands/intelligentMove/XMLRecordSaver.cpp
    src/model/basemodel/command/derivedCommands/intelligentMove/XMLRecordTypes.cpp
    src/model/basemodel/command/derivedCommands/IOManipulation.cpp
    src/model/basemodel/command/derivedCommands/Move.cpp
    src/model/basemodel/command/derivedCommands/MoveRelative.cpp
    src/model/basemodel/command/derivedCommands/Print.cpp
    src/model/basemodel/command/derivedCommands/VisionAct.cpp
    src/model/basemodel/command/derivedCommands/Wait.cpp
    src/model/basemodel/command/derivedCommands/WaitForCondition.cpp    
    src/model/basemodel/condition/Condition.cpp
    src/model/basemodel/condition/ConditionInstant.cpp
    src/model/basemodel/condition/ConditionMonitored.cpp
    src/model/basemodel/condition/conditions/DistanceComparison.cpp
    src/model/basemodel/condition/conditions/IoComparison.cpp
    src/model/basemodel/condition/conditions/IoComparisonMonitored.cpp
    src/model/basemodel/condition/conditions/PositionCartesianComparison.cpp
    src/model/basemodel/condition/conditions/PositionJointComparison.cpp
    src/model/basemodel/condition/ErrorCheck.cpp
    src/model/basemodel/condition/ErrorCheckMonitoredStart.cpp
    src/model/basemodel/controller/ControlUnit.cpp
    src/model/basemodel/controller/ControlUnitThreaded.cpp
    src/model/basemodel/controller/errorHandles/ErrorHandleDeterministic.cpp
    src/model/basemodel/controller/errorHandles/ErrorHandleInterface.cpp
    src/model/basemodel/controller/errorHandles/ErrorHandleRandom.cpp  
    src/model/basemodel/controller/errorHandles/ManualErrorInjecter.cpp
    src/model/basemodel/controller/messages/MessageEnvelope.cpp
    src/model/basemodel/controller/messages/MessagesError.cpp
    src/model/basemodel/controller/messages/MessagesContainer.cpp    
    src/model/basemodel/Identification.cpp
    src/model/basemodel/Instruction.cpp   
    src/model/basemodel/InstructionGraph.cpp     
    src/model/basemodel/InstructionUtilities.cpp    
    src/model/basemodel/State.cpp
    src/model/expandedmodel/derivedElements/ElementCommand.cpp
    src/model/expandedmodel/derivedElements/ElementErrorCheck.cpp
    src/model/expandedmodel/derivedElements/ElementErrorCheckMonitoredStart.cpp
    src/model/expandedmodel/derivedElements/ElementSequenceNested.cpp
    src/model/expandedmodel/Element.cpp 
    src/model/expandedmodel/intermediateModel/IntermediateModel.cpp
    src/model/expandedmodel/intermediateModel/IntermediateModelIns.cpp
    src/model/expandedmodel/intermediateModel/IntermediateModelLink.cpp
    src/model/expandedmodel/MemoryModel.cpp
    src/model/expandedmodel/Sequence.cpp
    src/common/common.cpp
    src/common/DataCollectPrint.cpp
    src/common/DataPrint.cpp
    src/common/GraphPrint.cpp
    src/common/GraphRepresentation.cpp
    src/common/ProjectedFrames.cpp
    src/common/WorkcellModel.cpp
    src/programs/getURJointconfiguration.cpp
    src/programs/MapKVMForceMode.cpp
    src/programs/MapKVM.cpp
    src/programs/MapKVMIntMove.cpp
    src/programs/MapTest.cpp
    src/programs/MapVola.cpp
    src/programs/TestDebug.cpp
    src/programs/TestDistanceCondition.cpp
	src/programs/TestIntActionDataExtraction.cpp
	src/programs/TestIntDataExtraction2.cpp
	src/programs/TestIntelligentMove.cpp
    src/programs/TestInterface.cpp
    src/programs/TestJoy.cpp
    src/programs/TestLanguage.cpp
    src/programs/TestLanguageXML.cpp
    src/programs/TestNewURInterface.cpp
    src/programs/TestNikolajInterface.cpp
    src/programs/TestPrint.cpp
    src/programs/TestServo.cpp
    src/programs/TestURReconnect.cpp
  
)

## Collection of librarys in a variable
set(
   alllibs
   rdsllib_v1
   carmen_hw_interface
   my_dsl_common_library
   ${catkin_LIBRARIES}
   ${ROBWORK_LIBRARIES}
   ${ROBWORKHARDWARE_LIBRARIES}
)

add_dependencies( 
	rdsllib_v1		
	carmen_hw_interface_generate_messages_cpp 
)

## Declare a cpp executable
add_executable( reversible_dsl_v1        src/main.cpp )

## Specify libraries to link a library or executable target against
target_link_libraries( reversible_dsl_v1    ${alllibs})


#############
## Install ##
#############

#############
## Testing ##
#############
