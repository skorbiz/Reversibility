/*
 * CommandInterpreter.cpp
 *
 *  Created on: Apr 5, 2017
 *      Author: josl
 */

#include "CommandInterpreter.h"

#include <d7cp_proxys/gripper/WSG25Proxy.h>
#include <d7cp_proxys/gripper/WSG25Gripper.h>
#include <d7cp_proxys/robot_movement_interface/RosProxy.h>

#include "CommandString.h"
#include "RosAutoSpinner.h"
#include "MemoryModel.hpp"
#include <../../reversible_dsl/src/D7cpWorkcell.h>
#include <../../reversible_dsl/src/Converter.h>
#include <../../reversible_dsl/src/Move.h>
#include <../../reversible_dsl/src/Move2Q.h>
#include <../../reversible_dsl/src/MoveLin.h>
#include <../../reversible_dsl/src/DynamicGrasp.h>
#include <../../reversible_dsl/src/skill/spiral_skill.hpp>
#include <../../reversible_dsl/src/skill/wiggle_skill.h>
#include <../../reversible_dsl/src/skill/Compliant_pih_skill.h>

//#include <rw/models/Device.hpp>
//#include <rw/models/WorkCell.hpp>
//#include <rw/kinematics/Kinematics.hpp>
//#include <rw/loaders/WorkCellLoader.hpp>
//#include <rwlibs/calibration/xml/XmlCalibrationLoader.hpp>
//#include <rwlibs/calibration/WorkCellCalibration.hpp>


CommandInterpreter::CommandInterpreter(int argc, char **argv, std::string nodeName) :
	wcPtr(std::make_shared<D7cpWorkcell>()),
	wc(*wcPtr.get()),
	variables_state("State"),
	variables_transforms("Transform3D"),
	variables_move("Move")

{
	std::cout << "Workcell model initiated" << std::endl;

	std::cout << "starting ros auto spinner" << std::endl;
	ros = std::make_shared<RosAutoSpinner>(argc, argv, nodeName);

	std::cout << "Starting WSG25" << std::endl;
	gripper::WSG25Proxy::Ptr wsgProxy = std::make_shared<gripper::WSG25Proxy>();
	gripper::WSG25Gripper::Ptr wsgInterface = std::make_shared<gripper::WSG25Gripper>(wsgProxy);
	wsg = std::make_shared<gripper::D7PC_Gripper>(wsgInterface);
	std::cout << "Starting Inspection system" << std::endl;
	inspectionSystem = std::make_shared<InspectionSystem>();
	std::cout << "Starting vision system" << std::endl;
	visionSystem = std::make_shared<VisionSystem>();
	std::cout << "Starting anyfeeder" << std::endl;
	anyfeeder = std::make_shared<Anyfeeder>();
	anyfeeder->start_any_feeder_action();
	anyfeeder->init_any_feeder_action();
	std::cout << "Starting robot_movement_interface::RosProxy" << std::endl;
	std::shared_ptr<robot_movement_interface::RosProxy> iiwaRosProxy = std::make_shared<robot_movement_interface::RosProxy>();
	std::cout << "Starting iiwa::RobworkInterface" << std::endl;
	iiwa = std::make_shared<robot_movement_interface::RobworkInterface>(iiwaRosProxy);
	std::cout << "Interfaces initiated" << std::endl;


	converter = wc.converter;

	rw::kinematics::State state0 = wc.getDefaultState();
	rw::math::Transform3D<> baseTcamera = converter->frameTframe(wc.baseFrame, wc.cameraFrame, state0);

	moveq = std::make_shared<Move2Q>(iiwa, wc.planner);
	movel = std::make_shared<MoveLin>(iiwa, wc.plannerRedundency, wc.toolFrame);
	dg = std::make_shared<DynamicGrasp>(baseTcamera);


	variable_graspConfig = wsg->outer_shell;
	variable_visType = VisionSystem::ObjectType::OuterShell;
	variable_insType = InspectionSystem::ObjectType::OuterShell;

	pinI = rand() % 3;
	pinJ = rand() % 3;


	std::cout << "Starting AnyfeederPickSkill" << std::endl;
	anyfeederPickSkill = std::make_shared<AnyfeederPickSkill>(wcPtr, wsg, visionSystem, anyfeeder, iiwa, baseTcamera);
	std::cout << "AnyfeederPickSkill initiated" << std::endl;

}

CommandInterpreter::~CommandInterpreter()
{
}

CommandInterpreter::ExecutionResult CommandInterpreter::execute(CommandString command, bool do_backwards)
{

	//////////////////////////////////////////////////////////////////////////
	// COMMENTS
	//////////////////////////////////////////////////////////////////////////
	if(command.command == "")
	{
	}
	else if(command.command.substr(0, 2) == "//")
	{
	}
	//////////////////////////////////////////////////////////////////////////
	// TEMPS
	//////////////////////////////////////////////////////////////////////////
	else if(command.command == "print state")
	{
		rw::kinematics::State& state = variables_state.getRef("state");
		std::cout << converter->toQ(state);
	}

	//////////////////////////////////////////////////////////////////////////
	// FLOW COMMANDS
	//////////////////////////////////////////////////////////////////////////
	else if(command.command == "restart_point")
	{
	}
	else if(command.command == "error")
	{
		ExecutionResult r;
		r.was_failure = true;
		r.error_description = "Induced error";
		return r;
	}

	//////////////////////////////////////////////////////////////////////////
	// LOCAL
	//////////////////////////////////////////////////////////////////////////
	else if(command.command == "local state")
	{
		assert(command.args.size() == 1);
		rw::kinematics::State s = wc.getDefaultState();
		if(do_backwards)
			variables_state.erase(command.args[0]);
		else
			variables_state.save(command.args[0], s);
	}
	else if(command.command == "local transform")
	{
		assert(command.args.size() == 1);
		rw::math::Transform3D<> t;
		if(do_backwards)
			variables_transforms.erase(command.args[0]);
		else
			variables_transforms.save(command.args[0], t);
	}
	else if(command.command == "local move")
	{
		assert(command.args.size() == 1);
		std::shared_ptr<Move> move = std::make_shared<Move>(iiwa, wc.planner, wc.wc, wc.device, wc.toolFrame);
		if(do_backwards)
			variables_move.erase(command.args[0]);
		else
			variables_move.save(command.args[0], move);
	}

	//////////////////////////////////////////////////////////////////////////
	// DELOCAL
	//////////////////////////////////////////////////////////////////////////
	else if(command.command == "delocal state")
	{
		assert(command.args.size() == 1);
		rw::kinematics::State s = wc.getDefaultState();

		if(do_backwards)
			variables_state.save(command.args[0], s);
		else
			variables_state.erase(command.args[0]);
	}
	else if(command.command == "delocal transform")
	{
		assert(command.args.size() == 2);
		if(do_backwards)
			variables_transforms.save(command.args[0], toTransform(command.args[1]));
		else
			variables_transforms.erase(command.args[0]);
	}
	else if(command.command == "delocal move")
	{
		assert(command.args.size() == 1);
		std::shared_ptr<Move> move = std::make_shared<Move>(iiwa, wc.planner, wc.wc, wc.device, wc.toolFrame);
		if(do_backwards)
			variables_move.save(command.args[0], move);
		else
			variables_move.erase(command.args[0]);
	}

	//////////////////////////////////////////////////////////////////////////
	// SET
	//////////////////////////////////////////////////////////////////////////
	else if(command.command == "set state" && command.args.size() == 3)
	{
		rw::kinematics::State& state = variables_state.getRef(command.args[0]);
		auto frame_move = toFrame(command.args[1]);
		auto frame_to = toFrame(command.args[2]);
		wc.attachTo(frame_move, frame_to, state);

	}
	else if(command.command == "set state" && command.args.size() == 2)
	{
		rw::kinematics::State& state = variables_state.getRef(command.args[0]);
		rw::math::Q qTo = toQ(command.args[1]);
		wc.setQ(qTo, state);

	}
	else if(command.command == "set state")
	{
		assert(false && "command.args.size() == 2 OR command.args.size() == 3");
	}
	else if(command.command == "set transform")
	{
		assert(command.args.size() == 2);
		rw::math::Transform3D<>& target = variables_transforms.getRef(command.args[0]);
		rw::math::Transform3D<> value = toTransform(command.args[1]);
		target = value;
	}

	//////////////////////////////////////////////////////////////////////////
	// STATIC VAR SET
	//////////////////////////////////////////////////////////////////////////

	else if(command.command == "set inspection_type")
	{
		assert(command.args.size() == 1);
		auto to = toInspectionObjectType(command.args[0]);
		variable_insType = to;
	}
	//////////////////////////////////////////////////////////////////////////
	else if(command.command == "set grasp_type")
	{
		assert(command.args.size() == 1);
		auto to = toGraspConfig(command.args[0]);
		variable_graspConfig = to;
	}
	//////////////////////////////////////////////////////////////////////////
	else if(command.command == "set vision_type")
	{
		assert(command.args.size() == 1);
		auto to = toVisionObjectType(command.args[0]);
		variable_visType = to;
	}
	//////////////////////////////////////////////////////////////////////////
	// GRIPPER
	//////////////////////////////////////////////////////////////////////////
	else if(command.command == "wsg open")
	{
		assert(command.args.size() == 0);

		if(do_backwards)
			wsg->close(variable_graspConfig);
		else
			wsg->open(variable_graspConfig);
	}
	else if(command.command == "wsg close")
	{
		assert(command.args.size() == 0);

		if(do_backwards)
			wsg->open(variable_graspConfig);
		else
			wsg->close(variable_graspConfig);
	}
	else if(command.command == "wsg test in_hand")
	{
		assert(command.args.size() == 0);
		if(do_backwards)
		{
		}
		else
		{
			bool was_in_hand = wsg->in_hand(variable_graspConfig);
			if(was_in_hand == false)
			{
				ROS_WARN("wsg in_hand unsucessfull");
				ExecutionResult r;
				r.was_failure = true;
				r.error_description = "wsg in_hand unsuccesfull";
				return r;
			}
		}
	}
	else if(command.command == "wsg test not_in_hand")
	{
		assert(command.args.size() == 0);
		if(do_backwards)
		{
		}
		else
		{
			bool was_in_hand = wsg->in_hand(variable_graspConfig);
			if(was_in_hand)
			{
				ROS_WARN("wsg in_hand unsucessfull");
				ExecutionResult r;
				r.was_failure = true;
				r.error_description = "wsg in_hand unsuccesfull";
				return r;
			}
		}
	}
	//////////////////////////////////////////////////////////////////////////
	// OTHER
	//////////////////////////////////////////////////////////////////////////

	else if(command.command == "sleep")
	{
		assert(command.args.size() == 1);
		double time = std::atof(command.args[0].c_str());
		ros::Duration(time).sleep();
	}
	else if(command.command == "inspection update")
	{
		assert(command.args.size() == 0);
		bool was_inspection_succesfull = inspectionSystem->update(variable_insType);

		if(do_backwards)
		{
		}
		else
		{
			if(was_inspection_succesfull == false)
			{
				ROS_WARN("Inspection unsuccesfull");
				ExecutionResult r;
				r.was_failure = true;
				r.error_description = "Inspection unsuccesfull";
				return r;
			}
		}
	}
	else if(command.command == "convert toBaseTtool")
	{
		assert(command.args.size() == 4);
		rw::math::Transform3D<>& target = variables_transforms.getRef(command.args[0]);
		rw::math::Transform3D<>& toolTobject = variables_transforms.getRef(command.args[1]);
		auto frame = toFrame(command.args[2]);
		rw::kinematics::State& state = variables_state.getRef(command.args[3]);

		target = converter->toBaseTtool(toolTobject, frame, state);
	}
	else if(command.command == "convert frame2frame")
	{
		assert(command.args.size() == 4);
		rw::math::Transform3D<>& target = variables_transforms.getRef(command.args[0]);
		auto from = toFrame(command.args[1]);
		auto to = toFrame(command.args[2]);
		rw::kinematics::State& state = variables_state.getRef(command.args[3]);

		target = converter->frameTframe(from, to, state);
	}

	//////////////////////////////////////////////////////////////////////////
	// MOVE PLANNING
	//////////////////////////////////////////////////////////////////////////
	else if(command.command == "move in_reach")
	{
		assert(command.args.size() == 3);
		std::shared_ptr<Move> move = variables_move.getRef(command.args[0]);
		rw::math::Transform3D<>& target = variables_transforms.getRef(command.args[1]);
		rw::kinematics::State& state = variables_state.getRef(command.args[2]);

		if(do_backwards)
		{
		}
		else
			if( !move->plan(target, state ) )
			{
				ROS_WARN("ptp move unsuccesfull");
				ExecutionResult r;
				r.was_failure = true;
				r.error_description = "Planning to fixture unsuccesfull";
				return r;
			}
	}

	else if(command.command == "move lin in_reach")
	{
		assert(command.args.size() == 2);
		rw::math::Transform3D<>& target = variables_transforms.getRef(command.args[0]);
		rw::kinematics::State& state = variables_state.getRef(command.args[1]);


		if(do_backwards)
		{
		}
		else
			if( !movel->is_in_reach(target, state ) )
			{
				ROS_WARN("lin move unsuccesfull");
				ExecutionResult r;
				r.was_failure = true;
				r.error_description = "Planning liniear to fixture unsuccesfull";
				return r;
			}

	}

	//////////////////////////////////////////////////////////////////////////
	// MOVE PLANNING
	//////////////////////////////////////////////////////////////////////////
	else if(command.command == "move")
	{
		assert(command.args.size() == 3);
		auto move = variables_move.getRef(command.args[0]);
		rw::math::Transform3D<>& toTarget = variables_transforms.getRef(command.args[1]);
		rw::kinematics::State& fromState = variables_state.getRef(command.args[2]);

		if( !move->plan(toTarget, fromState ) )
		{
			ROS_WARN("move ptp (2) unsucessfull");
			std::cout << dsl::color::RED << "move ptp (2) unsucessfull" << dsl::color::DEFAULT << std::endl;
			ExecutionResult r;
			r.was_failure = true;
			r.error_description = "Planning move unsuccesfull";
			return r;
		}
		if(do_backwards)
		{
			move->move_reverse();
		}
		else
			move->move_forward();
	}
	else if(command.command == "move lin")
	{
		assert(command.args.size() == 2);
		rw::math::Transform3D<>& to = variables_transforms.getRef(command.args[0]);
		rw::math::Transform3D<>& from = variables_transforms.getRef(command.args[1]);

		if(do_backwards)
			movel->move_forward(to);
		else
			movel->move_forward(from);
	}
	else if(command.command == "moveq")
	{
		assert(command.args.size() == 3);
		auto qFrom = toQ(command.args[0]);
		auto qTo = toQ(command.args[1]);
		rw::kinematics::State state = variables_state.getRef(command.args[2]);

		if(do_backwards)
		{
			wc.setQ(iiwa->getQ(), state);
			moveq->move(qFrom, state);
		}
		else
		{
			wc.setQ(iiwa->getQ(), state);
			moveq->move(qTo, state);
		}
	}
	else if(command.command == "move ptp")
	{
		assert(command.args.size() == 2);
		auto qFrom = toQ(command.args[0]);
		auto qTo = toQ(command.args[1]);

		if(do_backwards)
			iiwa->movePtp(qFrom);
		else
			iiwa->movePtp(qTo);
	}
	else if(command.command == "move retract_from_fixture")
	{
		assert(command.args.size() == 0);
		if(do_backwards)
		{
		}
		else
		{
			assert(command.args.size() == 1);
			auto move = variables_move.getRef(command.args[0]);
			move->move_retract_from_fixture();
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// ACTIONS AND SUPPORT
	//////////////////////////////////////////////////////////////////////////
	else if(command.command == "prepare feeder")
	{
		assert(command.args.size() == 1);
		auto objType = toAnyfeederPickSkillObjectType(command.args[0]);

		if(do_backwards)
		{
		}
		else
			anyfeederPickSkill->prepare(objType);

	}
	else if(command.command == "prepare feeder backward")
	{
		assert(command.args.size() == 1);
		auto objType = toAnyfeederPickSkillObjectType(command.args[0]);

		if(do_backwards)
		{
			anyfeederPickSkill->prepare(objType);
		}
		else
		{
		}
	}
	else if(command.command == "pick feeder")
	{
		assert(command.args.size() == 1);
		rw::kinematics::State state = variables_state.getRef(command.args[0]);

		if(do_backwards)
		{
			wc.setQ(iiwa->getQ(), state);
			moveq->move(wc.qDispenser, state);
			wsg->open(variable_graspConfig);
		}
		else
		{
			auto type = toAnyfeederPickSkillObjectType(variable_visType);
			anyfeederPickSkill->pick(type);
		}
	}
	//////////////////////////////////////////////////////////////////////////
	else if(command.command == "pick pin")
	{
		assert(command.args.size() == 1);
		rw::kinematics::State state = variables_state.getRef(command.args[0]);

		if(do_backwards)
		{
			wc.setQ(iiwa->getQ(), state);
			moveq->move(wc.qPinHoleAbove[pinI][pinJ], state);
			wsg->open(variable_graspConfig);
		}
		else
		{
			pinI = (pinI + 1) % 3;
			if(pinI == 0)
				pinJ = (pinJ + 1) % 3;

			wc.attachTo(wc.innerStructureAltColModelFrame, wc.assemblyFrame, state);
			wc.setQ(iiwa->getQ(), state);

			moveq->move(wc.qPinHoleAbove[pinI][pinJ], state);
			wc.setQ(iiwa->getQ(), state);

			rw::math::Transform3D<> baseTtoolPickRetreacted = converter->toBaseTtool(wc.qPinHoleAbove[pinI][pinJ], state);
			rw::math::Transform3D<> baseTtoolPick = converter->toBaseTtool(wc.qPinHole[pinI][pinJ], state);

			rw::math::Vector3D<> offset(0,0,0.001);
			rw::math::Transform3D<> toolToffset(offset);
			baseTtoolPickRetreacted = baseTtoolPickRetreacted*toolToffset;
			baseTtoolPick = baseTtoolPick*toolToffset;

			movel->move_forward(baseTtoolPick);
			ros::Duration(1).sleep();
			wsg->close(wsg->pin);
			ros::Duration(1).sleep();
			movel->move_forward(baseTtoolPickRetreacted);
		}
	}
	//////////////////////////////////////////////////////////////////////////
	else if(command.command == "pin skill")
	{
		assert(command.args.size() == 1);
		auto toolTobject = toTransform(command.args[0]);

		if(do_backwards)
		{
		}
		else
		{
			SpiralSkill spiral_skill(iiwa, wcPtr, toolTobject);
			bool was_spiral_succesful = spiral_skill.run_spiral_skill(SpiralSkill::sim_optimized_d7pc_parameters);

			if(!was_spiral_succesful)
			{
				ROS_WARN("SpiralSkill unsucessfull");
				ExecutionResult r;
				r.was_failure = true;
				r.error_description = "SpiralSkill unsuccesfull";
				return r;
			}

			WiggleSkill wiggle_skill(iiwa, wcPtr, toolTobject);
			bool was_wiggle_succesfull = wiggle_skill.run_skill(WiggleSkill::defaultParam());

			if(!was_wiggle_succesfull)
			{
				ROS_WARN("WiggleSkill unsucessfull");
				ExecutionResult r;
				r.was_failure = true;
				r.error_description = "WiggleSkill unsuccesfull";
				return r;
			}
		}
	}
	////////////////////////////////////////////////////////////////////////////////////////////
	else if(command.command == "innerstructure skill")
	{
		assert(command.args.size() == 3);
		auto toolTobject = toTransform(command.args[0]);
		auto baseTtoolAssemblyRetracted = toTransform(command.args[1]);
		auto state = variables_state.getRef(command.args[2]);

		Compliant_pih_skill skill(iiwa, wcPtr, toolTobject);

		if(do_backwards)
		{
			movel->move_forward(baseTtoolAssemblyRetracted);
		}
		else
		{
			std::vector<rw::math::Transform3D<> > vBaseTtool = skill.generateTrajectory_baseTtool(Compliant_pih_skill::d7pc_innerstructure);

			size_t i = 0;
			for(; i < vBaseTtool.size(); i++)
			{
				auto path = movel->to_path(vBaseTtool[i], state);
				if(path.size() == 0)
					break;
				wc.setQ(path.back(), state);
			}

			if(i != vBaseTtool.size())
			{
				ROS_WARN("innerstructure skill unsucessfull");
				ExecutionResult r;
				r.was_failure = true;
				r.error_description = "innerstructure skill unsuccesfull";
				return r;
			}

			skill.run_skill(Compliant_pih_skill::d7pc_innerstructure);
		}
	}


	////////////////////////////////////////////////////////////////////////////////////////////
	else
		assert(false &&  "Command was recognised");

	return ExecutionResult();
}

std::shared_ptr<rw::kinematics::Frame> CommandInterpreter::toFrame(std::string name)
{
	if(name == "wc.assemblyFrame")
		 return wc.assemblyFrame;
	if(name == "wc.toolFrame")
		 return wc.toolFrame;
	if(name == "wc.baseFrame")
		 return wc.baseFrame;

	if(name == "wc.outershellFrame")
		 return wc.outershellFrame;
	if(name == "wc.outershellDetachFrame")
		 return wc.outershellDetachFrame;
	if(name == "wc.outershellGraspFrame")
		 return wc.outershellGraspFrame;
	if(name == "wc.outershellAssemblyFrame")
		 return wc.outershellAssemblyFrame;
	if(name == "wc.outershellAssemblyRetractedFrame")
		 return wc.outershellAssemblyRetractedFrame;

	if(name == "wc.outershellAltColModelFrame")
		 return wc.outershellAltColModelFrame;
	if(name == "wc.outershellAltDetachFrame")
		 return wc.outershellAltDetachFrame;


	if(name == "wc.innerStructureFrame")
		 return wc.innerStructureFrame;
	if(name == "wc.innerStructureDetachFrame")
		 return wc.innerStructureDetachFrame;
	if(name == "wc.innerStructureGraspFrame")
		 return wc.innerStructureGraspFrame;
	if(name == "wc.innerStructureAssemblyFrame")
		 return wc.innerStructureAssemblyFrame;
	if(name == "wc.innerStructureAssemblyRetractedFrame")
		 return wc.innerStructureAssemblyRetractedFrame;
	if(name == "wc.innerStructureFixedPickTcpFrame")
		 return wc.innerStructureFixedPickTcpFrame;
	if(name == "wc.innerStructureFixedPickRetractedTcpFrame")
		 return wc.innerStructureFixedPickRetractedTcpFrame;

	if(name == "wc.innerStructureAltColModelFrame")
		 return wc.innerStructureAltColModelFrame;
	if(name == "wc.innerStructureAltDetachFrame")
		 return wc.innerStructureAltDetachFrame;

	if(name == "wc.thermoelementFrame")
		 return wc.thermoelementFrame;
	if(name == "wc.thermoelementDetachFrame")
		 return wc.thermoelementDetachFrame;
	if(name == "wc.thermoelementGraspFrame")
		 return wc.thermoelementGraspFrame;
	if(name == "wc.thermoelementAssemblyFrame")
		 return wc.thermoelementAssemblyFrame;
	if(name == "wc.thermoelementAssemblyRetractedFrame")
		 return wc.thermoelementAssemblyRetractedFrame;

	if(name == "wc.pinFrame")
		 return wc.pinFrame;
	if(name == "wc.pinDetachFrame")
		 return wc.pinDetachFrame;
	if(name == "wc.pinGraspFrame")
		 return wc.pinGraspFrame;
	if(name == "wc.pinAssemblyFrame")
		 return wc.pinAssemblyFrame;
	if(name == "wc.pinAssemblyRetractedFrame")
		 return wc.pinAssemblyRetractedFrame;

	if(name == "wc.springFrame")
		 return wc.springFrame;
	if(name == "wc.springDetachFrame")
		 return wc.springDetachFrame;
	if(name == "wc.springGraspFrame")
		 return wc.springGraspFrame;
	if(name == "wc.springAssemblyFrame")
		 return wc.springAssemblyFrame;
	if(name == "wc.springAssemblyRetractedFrame")
		 return wc.springAssemblyRetractedFrame;

	if(name == "wc.screwPartFrame")
		 return wc.screwPartFrame;
	if(name == "wc.screwPartDetachFrame")
		 return wc.screwPartDetachFrame;
	if(name == "wc.screwPartGraspFrame")
		 return wc.screwPartGraspFrame;
	if(name == "wc.screwPartAssemblyFrame")
		 return wc.screwPartAssemblyFrame;
	if(name == "wc.screwPartAssemblyRotatedFrame")
		 return wc.screwPartAssemblyRotatedFrame;
	if(name == "wc.screwPartAssemblyRetractedFrame")
		 return wc.screwPartAssemblyRetractedFrame;

	std::cerr << __PRETTY_FUNCTION__ << std::endl;
	std::cerr << "No frame with name: " << name << std::endl;
	exit(-1);
}

rw::math::Q CommandInterpreter::toQ(std::string name)
{
	if (variables_move.doContain(name))
		return variables_move.getRef(name)->get_endQ();
	if(name == "Q_IIWA")
		return iiwa->getQ();

	if(name == "wc.qInspectOuterShell")
		 return wc.qInspectOuterShell;
	if(name == "wc.qInspectInnerStructure")
		 return wc.qInspectInnerStructure;
	if(name == "wc.qInspectScrewPart")
		 return wc.qInspectScrewPart;
	if(name == "wc.qInspectPin")
		 return wc.qInspectPin;

	if(name == "wc.qDispenser")
		 return wc.qDispenser;
	if(name == "wc.qZero")
		 return wc.qZero;
	if(name == "wc.qAssemblySpringIntermediet")
		return wc.qAssemblySpringIntermediet;

	if(name == "wc.qPinHole")
		 return wc.qPinHole[pinI][pinJ];
	if(name == "wc.qPinHoleAbove")
		 return wc.qPinHoleAbove[pinI][pinJ];

	if(name == "wc.qSpringPickInit")
		 return wc.qSpringPickInit;
	if(name == "wc.qSpringPickPath1")
		 return wc.qSpringPickPath1;
	if(name == "wc.qSpringPickPath2")
		 return wc.qSpringPickPath2;

	if(name == "wc.qThermoelementPickInit")
		 return wc.qThermoelementPickInit;
	//if(name == "wc.qThermoelementPath1")
	//	 return wc.qThermoelementPath1;
	if(name == "wc.qThermoelementPath2")
		 return wc.qThermoelementPath2;
	if(name == "wc.qThermoelementEnd")
		 return wc.qThermoelementEnd;

	if(name == "wc.qPickRetractedInnerstrucutre")
		 return wc.qPickRetractedInnerstrucutre;

	std::cerr << __PRETTY_FUNCTION__ << std::endl;
	std::cerr << "No Q with name: " << name << std::endl;
	exit(-1);
}

rw::math::Transform3D<> CommandInterpreter::toTransform(std::string name)
{
	if (variables_transforms.doContain(name))
		return variables_transforms.getRef(name);
	if(name == "inspection.toolTobject")
		return inspectionSystem->calc_transform_tooTobject(variable_insType);
	if(name == "inspection.default_toolTobject")
		return inspectionSystem->get_toolTobject_ground_truth(variable_insType);
	if(name == "default")
		return rw::math::Transform3D<>();
	std::cerr << __PRETTY_FUNCTION__ << std::endl;
	std::cerr << "No Transform with name: " << name << std::endl;
	exit(-1);
}


gripper::D7PC_Gripper::GraspConfig CommandInterpreter::toGraspConfig(std::string name)
{
	if(name == "innerStructure")
		 return wsg->inner_structure;
	if(name == "outerShell")
		 return wsg->outer_shell;
	if(name == "thermoElement")
		 return wsg->thermo_element;
	if(name == "pin")
		 return wsg->pin;
	if(name == "spring")
		 return wsg->spring;
	if(name == "screwPart")
		 return wsg->skrew_part;

	std::cerr << __PRETTY_FUNCTION__ << std::endl;
	std::cerr << "No GraspConfig with name: " << name << std::endl;
	exit(-1);
}

VisionSystem::ObjectType CommandInterpreter::toVisionObjectType(std::string name)
{
	if(name == "outerShell")
		 return VisionSystem::ObjectType::OuterShell;
	if(name == "innerStructure")
		 return VisionSystem::ObjectType::InnerStructure;
	if(name == "screwPart")
		 return VisionSystem::ObjectType::ScrewPart;

	std::cerr << __PRETTY_FUNCTION__ << std::endl;
	std::cerr << "No Vision::ObjectType with name: " << name << std::endl;
	assert(false);
	return VisionSystem::ObjectType::OuterShell;
}

InspectionSystem::ObjectType CommandInterpreter::toInspectionObjectType(std::string name)
{
	if(name == "outerShell")
		 return InspectionSystem::ObjectType::OuterShell;
	if(name == "innerStructure")
		 return InspectionSystem::ObjectType::InnerStructure;
	if(name == "screwPart")
		 return InspectionSystem::ObjectType::ScrewPart;
	if(name == "pin")
		 return InspectionSystem::ObjectType::Pin;

	std::cerr << __PRETTY_FUNCTION__ << std::endl;
	std::cerr << "No InspectionSystem::ObjectType with name: " << name << std::endl;
	assert(false);
	return InspectionSystem::ObjectType::OuterShell;
}

AnyfeederPickSkill::ObjectType CommandInterpreter::toAnyfeederPickSkillObjectType(std::string name)
{
	if(name == "outerShell")
		 return AnyfeederPickSkill::ObjectType::OuterShell;
	if(name == "innerStructure")
		 return AnyfeederPickSkill::ObjectType::InnerStructure;
	if(name == "screwPart")
		 return AnyfeederPickSkill::ObjectType::ScrewPart;

	std::cerr << __PRETTY_FUNCTION__ << std::endl;
	std::cerr << "No AnyfeederPickSkill::ObjectType with name: " << name << std::endl;
	assert(false);
	return AnyfeederPickSkill::ObjectType::OuterShell;
}

AnyfeederPickSkill::ObjectType CommandInterpreter::toAnyfeederPickSkillObjectType(VisionSystem::ObjectType type)
{
	if(type == VisionSystem::ObjectType::OuterShell)
		 return AnyfeederPickSkill::ObjectType::OuterShell;
	if(type == VisionSystem::ObjectType::InnerStructure)
		 return AnyfeederPickSkill::ObjectType::InnerStructure;
	if(type == VisionSystem::ObjectType::ScrewPart)
		 return AnyfeederPickSkill::ObjectType::ScrewPart;

	std::cerr << __PRETTY_FUNCTION__ << std::endl;
	std::cerr << "No AnyfeederPickSkill::ObjectType from vis_type" << std::endl;
	assert(false);
	return AnyfeederPickSkill::ObjectType::OuterShell;
}

