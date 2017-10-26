/*
 * SequenceBuilder.hpp
 *
 *  Created on: Nov 17, 2014
 *      Author: josl
 */

#ifndef SEQUENCEBUILDER_HPP_
#define SEQUENCEBUILDER_HPP_

#include <iostream>
#include <rw/RobWork.hpp>

#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/URDefinitions.hpp>

#include <../src/model/basemodel/argument/quantities/IOPorts.hpp>
#include <../src/model/basemodel/argument/quantities/JointConfiguration.hpp>
#include <../src/model/basemodel/argument/quantities/Switch.hpp>

#include <../src/model/basemodel/BaseProgram.hpp>

#include <../src/model/basemodel/command/derivedCommands/intelligentMove/IntelligentActionModelInterface.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/IntelligentMoveModelInterface.hpp>
#include <../src/model/basemodel/command/derivedCommands/ActionFactory.hpp>
#include <../src/model/basemodel/command/derivedCommands/EmptyLinker.hpp>
#include <../src/model/basemodel/command/derivedCommands/ForceMode.hpp>
#include <../src/model/basemodel/command/derivedCommands/Grasp.hpp>
#include <../src/model/basemodel/command/derivedCommands/IOManipulation.hpp>
#include <../src/model/basemodel/command/derivedCommands/Move.hpp>
#include <../src/model/basemodel/command/derivedCommands/MoveRelative.hpp>
#include <../src/model/basemodel/command/derivedCommands/Print.hpp>
#include <../src/model/basemodel/command/derivedCommands/Wait.hpp>
#include <../src/model/basemodel/command/derivedCommands/WaitForCondition.hpp>

#include <../src/model/basemodel/condition/Condition.hpp>
#include <../src/model/basemodel/condition/ConditionInstant.hpp>
#include <../src/model/basemodel/condition/ConditionMonitored.hpp>
#include <../src/model/basemodel/condition/conditions/DistanceComparison.hpp>
#include <../src/model/basemodel/condition/conditions/IoComparison.hpp>
#include <../src/model/basemodel/condition/conditions/PositionCartesianComparison.hpp>
#include <../src/model/basemodel/condition/conditions/PositionJointComparison.hpp>

#include <../src/model/expandedmodel/derivedElements/ElementCommand.hpp>
#include <../src/model/expandedmodel/derivedElements/ElementErrorCheck.hpp>
#include <../src/model/expandedmodel/derivedElements/ElementErrorCheckMonitoredStart.hpp>
#include <../src/model/expandedmodel/derivedElements/ElementSequenceNested.hpp>

#include <../src/model/expandedmodel/Element.hpp>
#include <../src/model/expandedmodel/MemoryModel.hpp>
#include <../src/model/expandedmodel/Sequence.hpp>

#include <../src/model/basemodel/command/derivedCommands/intelligentMove/RecordDataset.hpp>



namespace dsl
{

class SequenceBuilder
{

public:
	SequenceBuilder();
	virtual ~SequenceBuilder();

	//Access functions
	dsl::BaseProgram getFinalProgram();
	std::shared_ptr<Sequence> getSeq(std::string name);

	//Sequence logic
	SequenceBuilder & sequence(std::string name); 						//Variations : operation / skill / macro
	SequenceBuilder & macro(std::string name); 							//
	SequenceBuilder & append(std::string name);							//Variations : apply / to all / overwrite
	SequenceBuilder & append(dsl::Argument * arg);						//
	SequenceBuilder & include(std::string name);						//Variations : Nest / use / call / include
	SequenceBuilder & nest(std::string name);							//
	SequenceBuilder & call(std::string name);							//

	//Reverse Logic
	SequenceBuilder & callReverse(std::string sequenceName);
	SequenceBuilder & uncall(std::string sequenceName);
	SequenceBuilder & reverseWith(std::string sequenceName);
	SequenceBuilder & inverses(std::string forwardSeq, std::string backwardSeq);
	SequenceBuilder & neverReversible();

	//Command primitives
	SequenceBuilder & cmd(std::shared_ptr<Element> element);
	SequenceBuilder & move(dsl::JointConfiguration q);
	SequenceBuilder & move(rw::math::Q q);
	SequenceBuilder & move(Vector3D<> direction, dsl::common::WorkcellModel::frame frame);
	SequenceBuilder & intMove(rw::math::Q qTo);
	SequenceBuilder & wait(double time);
	SequenceBuilder & wait(dsl::ConditionInstant * condition);
	SequenceBuilder & io(dsl::IOPorts ioPort, const dsl::Switch sw);
	SequenceBuilder & print(std::string text);
	SequenceBuilder & grasp(int angle);
	SequenceBuilder & gripper_open();
	SequenceBuilder & gripper_close();
	SequenceBuilder & forceMode(dsl::ForceModeArgument fm, dsl::ForceModeArgument fm_backward);
	SequenceBuilder & action(std::string filepathTrajectory, std::string frameTool, std::string frameFixture, dsl::JointConfiguration qInit);
	SequenceBuilder & intAction(std::string filepathTrajectory, std::string frameTool, std::string frameFixture, dsl::JointConfiguration qInit);

	//Conditions and error messages
	SequenceBuilder & check(dsl::ConditionInstant * condition);
	SequenceBuilder & check_start(dsl::ConditionMonitored * condition);
	SequenceBuilder & check_stop(dsl::ConditionMonitored * condition);

private:
	template<typename T>
	void nullptrErrorCheck(T var, std::string nameVar, std::string accespoint);

	std::shared_ptr<Element> _lastElement;
	std::shared_ptr<Sequence> _lastSequence;
	MemoryModel<std::shared_ptr<dsl::Sequence> > _sequences;

protected:
	//Common build variables
	rw::common::Ptr<ros::NodeHandle> _nodeHnd;
	std::shared_ptr<DigitalIOGeneralInterfaceProxy> _pDigitalIO;
	std::shared_ptr<DigitalIOGeneralInterfaceProxy> _pDigitalIOur;
	std::shared_ptr<RobotiqInterfaceProxy> 			_pRobotiq;

	std::shared_ptr<rwhw::UniversalRobotsRTLogging> _urrtInterface;
	std::shared_ptr<rwhw::URCallBackInterface> _urInterface;

	std::shared_ptr<dsl::intelligentmove::RecordDataset> _imoveDataset;
	std::shared_ptr<dsl::intelligentmove::RecordDataset> _iActionDataset;
	std::string _imoveDatasetPath;
	std::string _iActionDatasetPath;



};

// ******************************************************************
// **** TEMPLATE IMPLEMENTATIONS ************************************
// ******************************************************************

template<typename T>
void SequenceBuilder::nullptrErrorCheck(T var, std::string nameVar, std::string accespoint)
{
	if(var == nullptr)
	{
		std::cerr << "Nullptr error when accesing";
		std::cerr << " variable " << nameVar;
		std::cerr << " from SequenceBuilder::" << accespoint;
		std::cerr << std::endl;
		exit(-1);
	}
}



} /* namespace dsl */

#endif /* SEQUENCEBUILDER_HPP_ */
