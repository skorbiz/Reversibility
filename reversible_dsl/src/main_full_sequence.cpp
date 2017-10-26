/*
 * main.cpp
 *
 *  Created on: Jan 23, 2017
 *      Author: josl
 */

#include <memory>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <d7cp_proxys/gripper/WSG25Proxy.h>
#include <d7cp_proxys/gripper/WSG25Gripper.h>
#include <d7cp_proxys/gripper/D7PCGripper.h>
#include <d7cp_proxys/InspectionSystem.h>
#include <d7cp_proxys/VisionSystem.h>
#include <d7cp_proxys/Anyfeeder.h>

#include <d7cp_proxys/robot_movement_interface/RosProxy.h>
#include <d7cp_proxys/robot_movement_interface/RobworkInterface.h>

#include "skill/spiral_skill.hpp"
#include "skill/wiggle_skill.h"
#include "skill/Compliant_pih_skill.h"
#include "D7cpWorkcell.h"
#include "DynamicGrasp.h"
#include "colors.hpp"
#include "Move.h"
#include "MoveLin.h"
#include "Move2Q.h"

#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/calibration/xml/XmlCalibrationLoader.hpp>
#include <rwlibs/calibration/WorkCellCalibration.hpp>

class RosAutoSpin
{
public:
	bool _runSpinThread;
	boost::thread _spinThread;

	RosAutoSpin(int argc, char **argv)
	{
		ros::init(argc, argv, "reversible_dsl");
		ros::start();
		_runSpinThread = true;
		_spinThread = boost::thread(&RosAutoSpin::thread_func, this);
	}

	~RosAutoSpin()
	{
		_runSpinThread = false;
		_spinThread.join();
	}

	void thread_func()
	{
		while (_runSpinThread)
		{
			ros::spinOnce();
			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}
	}
};




int main(int argc, char **argv) 
{

	std::cout << "Strating Workcell initiation" << std::endl;
	std::shared_ptr<D7cpWorkcell> wcPtr = std::make_shared<D7cpWorkcell>();
	D7cpWorkcell& wc(*wcPtr.get());

	std::cout << "Workcell model initiated" << std::endl;


	RosAutoSpin thread(argc, argv);

	std::cout << "Starting WSG25" << std::endl;
	gripper::WSG25Proxy::Ptr wsgProxy = std::make_shared<gripper::WSG25Proxy>();
	gripper::WSG25Gripper::Ptr wsgInterface = std::make_shared<gripper::WSG25Gripper>(wsgProxy);
	gripper::D7PC_Gripper::Ptr wsg = std::make_shared<gripper::D7PC_Gripper>(wsgInterface);
	std::cout << "Starting Inspection system" << std::endl;
	std::shared_ptr<InspectionSystem> inspectionSystem = std::make_shared<InspectionSystem>();
	std::cout << "Starting vision system" << std::endl;
	std::shared_ptr<VisionSystem> visionSystem = std::make_shared<VisionSystem>();
	std::cout << "Starting anyfeeder" << std::endl;
	std::shared_ptr<Anyfeeder> anyfeeder = std::make_shared<Anyfeeder>();
	std::cout << "Starting robot_movement_interface::RosProxy" << std::endl;
	std::shared_ptr<robot_movement_interface::RosProxy> iiwaRosProxy = std::make_shared<robot_movement_interface::RosProxy>();
	std::cout << "Starting iiwa::RobworkInterface" << std::endl;
	std::shared_ptr<robot_movement_interface::RobworkInterface> iiwa = std::make_shared<robot_movement_interface::RobworkInterface>(iiwaRosProxy);
	std::cout << "Interfaces initiated" << std::endl;


	std::shared_ptr<Converter> converter = wc.converter;
	rw::kinematics::State state0 = wc.getDefaultState();
	rw::math::Transform3D<> baseTcamera = converter->frameTframe(wc.baseFrame, wc.cameraFrame, state0);

	Move2Q moveq(iiwa, wc.planner);
	MoveLin movel(iiwa, wc.plannerRedundency, wc.toolFrame);
	DynamicGrasp dg(baseTcamera);

	ros::Duration(2).sleep();


	/// PICK AND PLACE OUTERSHELL
	while(true)
	{
		rw::kinematics::State state = wc.getDefaultState();
		wc.setQ(iiwa->getQ(), state);
		moveq.move(wc.qDispenser, state);

		gripper::D7PC_Gripper::GraspConfig graspConfig = wsg->outer_shell;
		VisionSystem::ObjectType visType = VisionSystem::ObjectType::OuterShell;
		wsg->open(graspConfig);
		Move move1(iiwa, wc.planner, wc.wc, wc.device, wc.toolFrame);

		bool do_pick_from_anyfeeder = true;
		if(do_pick_from_anyfeeder)
		{
			while(ros::ok())
			{
				bool rv = visionSystem->update_pose(visType);
				if(rv == true)
				{
					dg.update(visionSystem->calc_camTobject());
					wc.setQ(iiwa->getQ(), state);
					bool r1 = move1.plan(dg.baseTtoolRetract, state);
					if(r1 == true)
					{
						wc.setQ(move1.get_endQ(), state);
						movel.set_expected_redundency(move1.get_endQ()[2]);
						bool r2 = movel.is_in_reach(dg.baseTtool, state);
						if(r2 == true)
							break;
						else
							std::cout << "Faild to plan linear path to object" << std::endl;
					}
					else
						std::cout << "Failed to plan path to object" << std::endl;
				}
				else
					std::cout << "Failed to located object with vision" << std::endl;
				anyfeeder->shakeFeeder();
			}
			move1.move_forward();

			movel.move_forward(dg.baseTtool);
			wsg->close(graspConfig);
			movel.move_forward(dg.baseTtoolRetract);
		}
		else
		{
			rw::math::Transform3D<> baseTpick = converter->frameTframe(wc.baseFrame, wc.outershellFixedPickTcpFrame, state);
			rw::math::Transform3D<> baseTpickRetracted = converter->frameTframe(wc.baseFrame, wc.outershellFixedPickRetractedTcpFrame, state);

//			wc.setQ(iiwa->getQ(), state);
//			moveq.move(wc.qZero, state);
			wc.setQ(iiwa->getQ(), state);

			bool r1 = move1.plan(baseTpickRetracted, state);
			if(r1 == false)
				continue;

			wc.setQ(move1.get_endQ(), state);
			movel.set_expected_redundency(move1.get_endQ()[2]);
			bool r2 = movel.is_in_reach(baseTpick, state);
			if(r2 == false)
				continue;

			move1.move_forward();
			movel.move_forward(baseTpick);
			wsg->close(graspConfig);
			movel.move_forward(baseTpickRetracted);

		}


		wc.setQ(iiwa->getQ(), state);
		wc.attachTo(wc.outershellFrame, wc.outershellGraspFrame, state);

		moveq.move(wc.qInspectOuterShell, state);
		bool was_inspection_succesfull = false;
		//ros::Duration(0.5).sleep();
		was_inspection_succesfull = inspectionSystem->update(InspectionSystem::ObjectType::OuterShell);
		if(was_inspection_succesfull == false)
		{
			ROS_INFO("Inspection unsuccesfull, restarting local loop");
			continue;
		}
		rw::math::Transform3D<> toolTobject = inspectionSystem->calc_transform_tooTobject(InspectionSystem::ObjectType::OuterShell);
		std::cout << dsl::color::GREEN << "deg: " << inspectionSystem->get_rotation_deg() << dsl::color::DEFAULT << std::endl;

		rw::math::Transform3D<> baseTtoolAssemblyRetracted = converter->toBaseTtool(toolTobject, wc.outershellAssemblyRetractedFrame, state);
		rw::math::Transform3D<> baseTtoolAssembly = converter->toBaseTtool(toolTobject, wc.outershellAssemblyFrame, state);

		Move move4(iiwa, wc.planner, wc.wc, wc.device, wc.toolFrame);
		wc.setQ(iiwa->getQ(), state);
		if( !move4.plan(baseTtoolAssemblyRetracted, state ) )
		{
			ROS_INFO("Planning to fixture unsuccesfull, restarting local loop");
			continue;
		}

		wc.setQ(move4.get_endQ(), state);
		movel.set_expected_redundency(move4.get_endQ()[2]);
		if( !movel.is_in_reach(baseTtoolAssembly, state ) )
		{
			ROS_INFO("Planning insertion unsuccesfull, restarting local loop");
			continue;
		}

		move4.move_forward();
		movel.move_forward(baseTtoolAssembly);
		wsg->open(wsg->outer_shell);
		movel.move_forward(baseTtoolAssemblyRetracted);
		break;
	}

	/// PICK AND INNER_STRUCTURE
	while(true)
	{
		rw::kinematics::State state = wc.getDefaultState();
		wc.attachTo(wc.outershellAltColModelFrame, wc.assemblyFrame, state);
		wc.setQ(iiwa->getQ(), state);
		moveq.move(wc.qDispenser, state);

		gripper::D7PC_Gripper::GraspConfig graspConfig = wsg->inner_structure;
		VisionSystem::ObjectType visType = VisionSystem::ObjectType::InnerStructure;
		wsg->open(graspConfig);
		Move move1(iiwa, wc.planner, wc.wc, wc.device, wc.toolFrame);

		bool do_pick_from_anyfeeder = false;
		if(do_pick_from_anyfeeder)
		{
			while(ros::ok())
			{
				bool rv = visionSystem->update_pose(visType);
				if(rv == true)
				{
					dg.update(visionSystem->calc_camTobject());
					wc.setQ(iiwa->getQ(), state);
					bool r1 = move1.plan(dg.baseTtoolRetract, state);
					if(r1 == true)
					{
						wc.setQ(move1.get_endQ(), state);
						bool r2 = movel.is_in_reach(dg.baseTtool, state);
						if(r2 == true)
							break;
						else
							std::cout << "Faild to plan linear path to object" << std::endl;
					}
					else
						std::cout << "Failed to plan path to object" << std::endl;
				}
				else
					std::cout << "Failed to located object with vision" << std::endl;

				anyfeeder->shakeFeeder();
			}
			move1.move_forward();
			movel.set_expected_redundency(move1.get_endQ()[2]);
			movel.move_forward(dg.baseTtool);
			wsg->close(graspConfig);
			movel.move_forward(dg.baseTtoolRetract);

		}
		else
		{
			rw::math::Transform3D<> baseTpick = converter->frameTframe(wc.baseFrame, wc.innerStructureFixedPickTcpFrame, state);
			rw::math::Transform3D<> baseTpickRetracted = converter->frameTframe(wc.baseFrame, wc.innerStructureFixedPickRetractedTcpFrame, state);

//			wc.setQ(iiwa->getQ(), state);
//			moveq.move(wc.qZero, state);
			wc.setQ(iiwa->getQ(), state);

			bool r1 = move1.plan(baseTpickRetracted, state);
			if(r1 == false)
				continue;

			wc.setQ(move1.get_endQ(), state);
			bool r2 = movel.is_in_reach(baseTpick, state);
			if(r2 == false)
				continue;

			move1.move_forward();
			movel.move_forward(baseTpick);
			wsg->close(graspConfig);
			movel.move_forward(baseTpickRetracted);
		}

		wc.setQ(iiwa->getQ(), state);
		wc.attachTo(wc.innerStructureFrame, wc.innerStructureGraspFrame, state);
		moveq.move(wc.qInspectInnerStructure, state);

		bool was_inspection_succesfull = false;
		//ros::Duration(0.5).sleep();
		was_inspection_succesfull = inspectionSystem->update(InspectionSystem::ObjectType::InnerStructure);
		if(was_inspection_succesfull == false)
		{
			ROS_INFO("Inspection unsuccesfull, restarting local loop");
			continue;
		}
		rw::math::Transform3D<> toolTobject = inspectionSystem->calc_transform_tooTobject(InspectionSystem::ObjectType::InnerStructure);
		std::cout << dsl::color::GREEN << "deg: " << inspectionSystem->get_rotation_deg() << dsl::color::DEFAULT << std::endl;

		Move move3(iiwa, wc.planner, wc.wc, wc.device, wc.toolFrame);

		rw::math::Transform3D<> baseTtoolAssemblyRetracted = converter->toBaseTtool(toolTobject, wc.innerStructureAssemblyRetractedFrame, state0 );
		rw::math::Transform3D<> baseTtoolAssembly = converter->toBaseTtool(toolTobject, wc.innerStructureAssemblyFrame, state0 );

		wc.setQ(iiwa->getQ(), state);
		if( !move3.plan(baseTtoolAssemblyRetracted, state ) )
			continue;

		wc.attachTo(wc.innerStructureFrame, wc.innerStructureDetachFrame, state);
		wc.setQ(move3.get_endQ(), state);
		movel.set_expected_redundency(move3.get_endQ()[2]);
		if( !movel.is_in_reach(baseTtoolAssembly, state ) )
			continue;

		Compliant_pih_skill skill(iiwa, wcPtr, toolTobject);

		//Test line skill moves
		{
			std::vector<rw::math::Transform3D<> > vBaseTtool = skill.generateTrajectory_baseTtool(Compliant_pih_skill::d7pc_innerstructure);

			size_t i = 0;
			for(; i < vBaseTtool.size(); i++)
			{
				auto path = movel.to_path(vBaseTtool[i], state);
				if(path.size() == 0)
					break;
				wc.setQ(path.back(), state);
			}

			if(i != vBaseTtool.size())
			{
				std::cout << "Faild to plan linear skill path in skill step: " << i << std::endl;
				continue;
			}

		}


		move3.move_forward();
		skill.run_skill(Compliant_pih_skill::d7pc_innerstructure);
		//movel.move_forward(baseTtoolAssembly);
		wsg->open(wsg->inner_structure);
		movel.move_forward(baseTtoolAssemblyRetracted);
//		break;
	}


	///// PICK AND PLACE THERMO ELEMENT
	{
		rw::kinematics::State state = wc.getDefaultState();
		wc.attachTo(wc.innerStructureAltColModelFrame, wc.assemblyFrame, state);
		wsg->open(wsg->thermo_element);

		wc.setQ(iiwa->getQ(), state);
		moveq.move(wc.qThermoelementPickInit, state);
		iiwa->movePtp(wc.qThermoelementPath2);
		wsg->close(wsg->thermo_element);
		iiwa->movePtp(wc.qThermoelementEnd);


		wc.setQ(iiwa->getQ(), state);
		wc.attachTo(wc.thermoelementFrame, wc.thermoelementGraspFrame, state);
		rw::math::Transform3D<> toolTobject = converter->frameTframe(wc.toolFrame, wc.thermoelementGraspFrame, state);

		Move move1(iiwa, wc.planner, wc.wc, wc.device, wc.toolFrame);
		rw::math::Transform3D<> baseTtoolAssembly = converter->toBaseTtool(toolTobject, wc.thermoelementAssemblyFrame, state0 );
		rw::math::Transform3D<> baseTtoolAssemblyRetracted = converter->toBaseTtool(toolTobject, wc.thermoelementAssemblyRetractedFrame, state0 );

		while(true)
		{
			bool r = move1.plan(baseTtoolAssemblyRetracted, state );
			if(r == true)
				break;
			else
				ROS_WARN("Faild to plan path to drop thermo element, retrying");
		}
		move1.move_forward();
		movel.move_forward(baseTtoolAssembly);
		wsg->open(wsg->thermo_element);
		movel.move_forward(baseTtoolAssemblyRetracted);
	}


	///// PIN PICK
	int pinI = rand() % 3;
	int pinJ = rand() % 3;
	while(true)
	{
		pinI = (pinI + 1) % 3;
		if(pinI == 0)
			pinJ = (pinJ + 1) % 3;


		rw::kinematics::State state = wc.getDefaultState();
		wc.attachTo(wc.innerStructureAltColModelFrame, wc.assemblyFrame, state);
		wc.setQ(iiwa->getQ(), state);

		moveq.move(wc.qPinHoleAbove[pinI][pinJ], state);
		wc.setQ(iiwa->getQ(), state);
		wsg->open(wsg->pin);

		rw::math::Transform3D<> baseTtoolPickRetreacted = converter->toBaseTtool(wc.qPinHoleAbove[pinI][pinJ], state);
		rw::math::Transform3D<> baseTtoolPick = converter->toBaseTtool(wc.qPinHole[pinI][pinJ], state);

		rw::math::Vector3D<> offset(0,0,0.001);
		rw::math::Transform3D<> toolToffset(offset);
		baseTtoolPickRetreacted = baseTtoolPickRetreacted*toolToffset;
		baseTtoolPick = baseTtoolPick*toolToffset;

		movel.move_forward(baseTtoolPick);
		//ros::Duration(1).sleep();
		wsg->close(wsg->pin);
		//ros::Duration(1).sleep();
		movel.move_forward(baseTtoolPickRetreacted);

		wc.setQ(iiwa->getQ(), state);
		wc.attachTo(wc.pinFrame, wc.pinGraspFrame, state);
		moveq.move(wc.qInspectPin, state);
		//ros::Duration(0.5).sleep();
		bool was_inspection_succesfull = inspectionSystem->update(InspectionSystem::ObjectType::Pin);
		if(was_inspection_succesfull == false)
		{
			ROS_INFO("Inspection unsuccesfull, restarting local loop");
			continue;
		}

		rw::math::Transform3D<> toolTobject = inspectionSystem->calc_transform_tooTobject(InspectionSystem::ObjectType::Pin);
		std::cout << dsl::color::GREEN << "deg: " << inspectionSystem->get_rotation_deg() << dsl::color::DEFAULT << std::endl;

//		wc.setQ(iiwa->getQ(), state);
//		moveq.move(wc.qZero, state);
		wc.setQ(iiwa->getQ(), state);

		Move move2(iiwa, wc.planner, wc.wc, wc.device, wc.toolFrame);

//		rw::math::Transform3D<> toolTobject = inspectionSystem->get_toolTobject_ground_truth(InspectionSystem::ObjectType::Pin);

		rw::math::Transform3D<> baseTtoolAssemblyRetracted = converter->toBaseTtool(toolTobject, wc.pinAssemblyRetractedFrame, state0 );
		rw::math::Transform3D<> baseTtoolAssembly = converter->toBaseTtool(toolTobject, wc.pinAssemblyFrame, state0 );


		wc.setQ(iiwa->getQ(), state);
		if( !move2.plan(baseTtoolAssemblyRetracted, state ) )
			continue;

		wc.setQ(move2.get_endQ(), state);
		if( !movel.is_in_reach(baseTtoolAssembly, state ) )
			continue;

		move2.move_forward();

		SpiralSkill spiral_skill(iiwa, wcPtr, toolTobject);
		spiral_skill.run_spiral_skill(SpiralSkill::sim_optimized_d7pc_parameters);

		WiggleSkill wiggle_skill(iiwa, wcPtr, toolTobject);
		wiggle_skill.run_skill(WiggleSkill::defaultParam());

		wsg->open(wsg->pin);
		movel.move_forward(baseTtoolAssemblyRetracted);
//		break;
	}

	///// PICK AND PLACE SPRING
	{
		rw::kinematics::State state = wc.getDefaultState();
		wc.attachTo(wc.innerStructureAltColModelFrame, wc.assemblyFrame, state);
		wsg->open(wsg->spring);

		wc.setQ(iiwa->getQ(), state);
		moveq.move(wc.qSpringPickInit, state);
		iiwa->movePtp(wc.qSpringPickPath1);
		iiwa->movePtp(wc.qSpringPickPath2);
		wsg->close(wsg->spring);
		iiwa->movePtp(wc.qSpringPickInit);

		//		wc.setQ(iiwa->getQ(), state);
		//		moveq.move(wc.qZero, state);
		wc.setQ(iiwa->getQ(), state);

		rw::math::Transform3D<> toolTobject = converter->frameTframe(wc.toolFrame, wc.springGraspFrame, state);
		rw::math::Transform3D<> baseTtoolAssembly = converter->toBaseTtool(toolTobject, wc.springAssemblyFrame, state0 );
		rw::math::Transform3D<> baseTtoolAssemblyRetracted = converter->toBaseTtool(toolTobject, wc.springAssemblyRetractedFrame, state0 );

		Move move1(iiwa, wc.planner, wc.wc, wc.device, wc.toolFrame);
		while(true)
		{
			bool r = move1.plan(baseTtoolAssemblyRetracted, state );
			wc.setQ(move1.get_endQ(), state);
			movel.is_in_reach(baseTtoolAssembly, state);

			if(r == true)
				break;
			else
				ROS_WARN("Faild to plan path to drop spring element, retrying");

		}



		move1.move_forward();
		movel.move_forward(baseTtoolAssembly);
		wsg->open(wsg->spring);
		movel.move_forward(baseTtoolAssemblyRetracted);
		move1.move_retract_from_fixture();
	}

	///// PICK AND SCREW_PART
	while(true)
	{
		rw::kinematics::State state = wc.getDefaultState();
		wc.attachTo(wc.innerStructureAltColModelFrame, wc.assemblyFrame, state);
		wc.setQ(iiwa->getQ(), state);

		if(moveq.move(wc.qDispenser, state) == false)
			continue;

		gripper::D7PC_Gripper::GraspConfig graspConfig = wsg->skrew_part;
		VisionSystem::ObjectType visType = VisionSystem::ObjectType::ScrewPart;
		wsg->open(graspConfig);
		Move move1(iiwa, wc.planner, wc.wc, wc.device, wc.toolFrame);
		while(ros::ok())
		{
			bool rv = visionSystem->update_pose(visType);

			if(rv == true)
			{
				dg.update(visionSystem->calc_camTobject());
				wc.setQ(iiwa->getQ(), state);
				bool r1 = move1.plan(dg.baseTtoolRetract, state);
				if(r1 == true)
				{
					wc.setQ(move1.get_endQ(), state);
					movel.set_expected_redundency(move1.get_endQ()[2]);
					bool r2 = movel.is_in_reach(dg.baseTtool, state);
					if(r2 == true)
						break;
				}
			}
			anyfeeder->shakeFeeder();
		}
		move1.move_forward();
		movel.move_forward(dg.baseTtool);
		wsg->close(graspConfig);
		movel.move_forward(dg.baseTtoolRetract);


		wc.setQ(iiwa->getQ(), state);
		wc.attachTo(wc.screwPartFrame, wc.screwPartGraspFrame, state);
		moveq.move(wc.qInspectScrewPart, state);

		//ros::Duration(0.5).sleep();
		bool was_inspection_succesfull = inspectionSystem->update(InspectionSystem::ObjectType::ScrewPart);
		if(was_inspection_succesfull == false)
		{
			ROS_INFO("Inspection unsuccesfull, restarting local loop");
			continue;
		}
		rw::math::Transform3D<> toolTobject = inspectionSystem->calc_transform_tooTobject(InspectionSystem::ObjectType::ScrewPart);
		std::cout << dsl::color::GREEN << "deg: " << inspectionSystem->get_rotation_deg() << dsl::color::DEFAULT << std::endl;

		rw::math::Transform3D<> baseTtoolAssembly = converter->toBaseTtool(toolTobject, wc.screwPartAssemblyFrame, state0 );
		rw::math::Transform3D<> baseTtoolAssemblyRetracted = converter->toBaseTtool(toolTobject, wc.screwPartAssemblyRetractedFrame, state0 );

		wc.setQ(iiwa->getQ(), state);

		Move move3(iiwa, wc.planner, wc.wc, wc.device, wc.toolFrame);
		if( !move3.plan(baseTtoolAssemblyRetracted, state ) )
			continue;

		wc.setQ(move3.get_endQ(), state);
		movel.set_expected_redundency(move3.get_endQ()[2]);
		if(!movel.is_in_reach(baseTtoolAssembly, state))
			continue;

		move3.move_forward();
		movel.move_forward(baseTtoolAssembly);
		wsg->open(wsg->skrew_part);
		movel.move_forward(baseTtoolAssemblyRetracted);
		break;
	}

	return 0;
}
