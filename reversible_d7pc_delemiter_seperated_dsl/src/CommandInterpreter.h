/*
 * CommandInterpreter.h
 *
 *  Created on: Apr 5, 2017
 *      Author: josl
 */

#ifndef REVERSIBLE_ASSEMBLY_SRC_COMMANDINTERPRETER_H_
#define REVERSIBLE_ASSEMBLY_SRC_COMMANDINTERPRETER_H_

#include <memory>
#include <d7cp_proxys/robot_movement_interface/RobworkInterface.h>
#include <d7cp_proxys/gripper/D7PCGripper.h>
#include <d7cp_proxys/InspectionSystem.h>
#include <d7cp_proxys/VisionSystem.h>
#include <d7cp_proxys/Anyfeeder.h>
#include <../../reversible_dsl/src/skill/AnyfeederPickSkill.h>


#include "MemoryModel.hpp"

#include <rw/kinematics/State.hpp>

class CommandString;
class RosAutoSpinner;
class Converter;
class D7cpWorkcell;
class Move;
class Move2Q;
class MoveLin;
class DynamicGrasp;

class CommandInterpreter
{

struct ExecutionResult
{
	bool was_failure;
	bool was_failure_critical;
	std::string error_description;
	ExecutionResult() : was_failure(false), was_failure_critical(false), error_description(""){};
};

public:
	CommandInterpreter(int argc, char **argv, std::string nodeName);
	virtual ~CommandInterpreter();

	ExecutionResult execute(CommandString command, bool do_backwards = false);

private:
	std::shared_ptr<rw::kinematics::Frame> toFrame(std::string name);
	rw::math::Transform3D<> toTransform(std::string name);
	rw::math::Q toQ(std::string name);
	gripper::D7PC_Gripper::GraspConfig toGraspConfig(std::string name);
	VisionSystem::ObjectType toVisionObjectType(std::string name);
	InspectionSystem::ObjectType toInspectionObjectType(std::string name);
	AnyfeederPickSkill::ObjectType toAnyfeederPickSkillObjectType(std::string name);
	AnyfeederPickSkill::ObjectType toAnyfeederPickSkillObjectType(VisionSystem::ObjectType);

private:
	std::shared_ptr<D7cpWorkcell> wcPtr;
	D7cpWorkcell& wc;

	std::shared_ptr<RosAutoSpinner> ros;
	gripper::D7PC_Gripper::Ptr wsg;
	std::shared_ptr<InspectionSystem> inspectionSystem;
	std::shared_ptr<VisionSystem> visionSystem;
	std::shared_ptr<Anyfeeder> anyfeeder;
	std::shared_ptr<robot_movement_interface::RobworkInterface> iiwa;
	std::shared_ptr<Converter> converter;
	std::shared_ptr<AnyfeederPickSkill> anyfeederPickSkill;

	std::shared_ptr<Move2Q> moveq;
	std::shared_ptr<MoveLin> movel;
	std::shared_ptr<DynamicGrasp> dg;

	MemoryModel<rw::kinematics::State> variables_state;
	MemoryModel<rw::math::Transform3D<> > variables_transforms;
	MemoryModel<std::shared_ptr<Move> > variables_move;

	gripper::D7PC_Gripper::GraspConfig variable_graspConfig;
	InspectionSystem::ObjectType variable_insType;
	VisionSystem::ObjectType variable_visType;

	int pinI;
	int pinJ;





};

#endif /* REVERSIBLE_ASSEMBLY_SRC_COMMANDINTERPRETER_H_ */
