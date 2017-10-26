/*
 * AnyfeederPickSkill.cpp
 *
 *  Created on: Apr 21, 2017
 *      Author: josl
 */

#include "AnyfeederPickSkill.h"

AnyfeederPickSkill::AnyfeederPickSkill(
		std::shared_ptr<D7cpWorkcell> wc,
		std::shared_ptr<gripper::D7PC_Gripper> wsg,
		std::shared_ptr<VisionSystem> visionSystem,
		std::shared_ptr<Anyfeeder> anyfeeder,
		std::shared_ptr<robot_movement_interface::RobworkInterface> iiwa,
		rw::math::Transform3D<> baseTcamera
		) :
	wc(wc),
	wsg(wsg),
	visionSystem(visionSystem),
	anyfeeder(anyfeeder),
	iiwa(iiwa),
	plannerTnt(std::make_shared<KukaIIWAPlanner>(wc->wc, "KukaIIWA")),
	plannerTntRedundency(std::make_shared<KukaIIWAPlanner>(wc->wc, "KukaIIWA", 2)),
	planner(std::make_shared<Planner>(plannerTnt, wc->wc, wc->device)),
	plannerRedundency(std::make_shared<Planner>(plannerTntRedundency, wc->wc, wc->device)),
	move(iiwa, planner, wc->wc, wc->device, wc->toolFrame),
	moveq(iiwa, planner),
	movel(iiwa, plannerRedundency, wc->toolFrame),
	dg(baseTcamera),
	object_desired(ObjectType::NoObject),
	object_located(ObjectType::NoObject)
{
	runSpinThread = true;
	spinThread = boost::thread(&AnyfeederPickSkill::run, this);


}

AnyfeederPickSkill::~AnyfeederPickSkill()
{
	runSpinThread = false;
	spinThread.join();
}

void AnyfeederPickSkill::run()
{
	while (runSpinThread)
	{
		if( object_desired != object_located )
		{
			std::lock_guard<std::mutex> lock(mutex);
			readySkill(object_desired);
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	}

}

void AnyfeederPickSkill::prepare(ObjectType type)
{
	object_desired = type;
}

void AnyfeederPickSkill::pick(ObjectType type)
{
	prepare(type);

	while(object_desired != object_located)
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));

	std::lock_guard<std::mutex> lock(mutex);
	executeSkill();
	object_desired = ObjectType::NoObject;
	object_located = ObjectType::NoObject;

}




void AnyfeederPickSkill::readySkill(ObjectType object)
{

	std::cout << dsl::color::BOLDYELLOW << "Planning to " << toString(object) << dsl::color::DEFAULT << std::endl;

	rw::kinematics::State state = wc->getDefaultState();

	bool rv = visionSystem->update_pose(toVisionObjectType(object));

	if(rv == true)
	{
		dg.update(visionSystem->calc_camTobject());
		wc->setQ(wc->qDispenser, state);
		bool r1 = move.plan(dg.baseTtoolRetract, state);
		if(r1 == true)
		{
			wc->setQ(move.get_endQ(), state);
			movel.set_expected_redundency(move.get_endQ()[2]);
			bool r2 = movel.is_in_reach(dg.baseTtool, state);
			if(r2 == true)
			{
				std::cout << dsl::color::BOLDYELLOW << "Succefully planed to " << toString(object) << dsl::color::DEFAULT << std::endl;
				object_located = object;
				return;
			}
			else
				std::cout << dsl::color::BOLDYELLOW << "Faild to plan linear path to " << toString(object) << dsl::color::DEFAULT << std::endl;
		}
		else
			std::cout<< dsl::color::BOLDYELLOW << "Failed to plan path to " << toString(object) << dsl::color::DEFAULT<< std::endl;
	}
	else
		std::cout<< dsl::color::BOLDYELLOW << "Failed to located " << toString(object) << " with vision" << dsl::color::DEFAULT<< std::endl;

	anyfeeder->shakeFeeder();
	object_located = ObjectType::NoObject;
}

void AnyfeederPickSkill::executeSkill()
{
	move.move_forward();
	movel.move_forward(dg.baseTtool);
	wsg->close(toGraspConfig(object_located));
	movel.move_forward(dg.baseTtoolRetract);
}



gripper::D7PC_Gripper::GraspConfig AnyfeederPickSkill::toGraspConfig(ObjectType type)
{
	if(type == ObjectType::InnerStructure)
		 return wsg->inner_structure;
	if(type == ObjectType::OuterShell)
		 return wsg->outer_shell;
	if(type == ObjectType::ScrewPart)
		 return wsg->skrew_part;

	std::cerr << __PRETTY_FUNCTION__ << std::endl;
	std::cerr << "No GraspConfig of given type" << std::endl;
	exit(-1);
}

VisionSystem::ObjectType AnyfeederPickSkill::toVisionObjectType(ObjectType type)
{
	if(type == ObjectType::OuterShell)
		 return VisionSystem::ObjectType::OuterShell;
	if(type == ObjectType::InnerStructure)
		 return VisionSystem::ObjectType::InnerStructure;
	if(type == ObjectType::ScrewPart)
		 return VisionSystem::ObjectType::ScrewPart;

	std::cerr << __PRETTY_FUNCTION__ << std::endl;
	std::cerr << "No Vision::ObjectType of given type" << std::endl;
	exit(-1);
}


std::string AnyfeederPickSkill::toString(ObjectType type)
{
	if(type == ObjectType::OuterShell)
		 return "outershell";
	if(type == ObjectType::InnerStructure)
		 return "innerstructure";
	if(type == ObjectType::ScrewPart)
		 return "screwpart";
	return "NoObject";
}

