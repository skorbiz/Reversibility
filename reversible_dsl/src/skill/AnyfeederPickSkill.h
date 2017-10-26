/*
 * AnyfeederPickSkill.h
 *
 *  Created on: Apr 21, 2017
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_SKILL_ANYFEEDERPICKSKILL_H_
#define REVERSIBLE_DSL_SRC_SKILL_ANYFEEDERPICKSKILL_H_

#include <memory>
#include <boost/thread.hpp>

#include <rw/math/Transform3D.hpp>

#include <d7cp_proxys/gripper/D7PCGripper.h>
#include <d7cp_proxys/VisionSystem.h>
#include <d7cp_proxys/Anyfeeder.h>
#include <d7cp_proxys/robot_movement_interface/RosProxy.h>
#include <d7cp_proxys/robot_movement_interface/RobworkInterface.h>

#include "../D7cpWorkcell.h"
#include "../DynamicGrasp.h"
#include "../colors.hpp"
#include "../Move.h"
#include "../MoveLin.h"
#include "../Move2Q.h"


class AnyfeederPickSkill
{

public:
	enum class ObjectType { OuterShell,	InnerStructure,	ScrewPart, NoObject };

	AnyfeederPickSkill(
			std::shared_ptr<D7cpWorkcell> wc,
			std::shared_ptr<gripper::D7PC_Gripper> wsg,
			std::shared_ptr<VisionSystem> visionSystem,
			std::shared_ptr<Anyfeeder> anyfeeder,
			std::shared_ptr<robot_movement_interface::RobworkInterface> iiwa,
			rw::math::Transform3D<> baseTcamera);
	virtual ~AnyfeederPickSkill();

	void prepare(ObjectType);
	void pick(ObjectType type);


private:
	void run();
	void readySkill(ObjectType type);
	void executeSkill();
	gripper::D7PC_Gripper::GraspConfig toGraspConfig(ObjectType type);
	VisionSystem::ObjectType toVisionObjectType(ObjectType type);
	std::string toString(ObjectType type);



private:
	bool runSpinThread;
	boost::thread spinThread;
	std::mutex mutex;

	std::shared_ptr<D7cpWorkcell> wc;
	std::shared_ptr<gripper::D7PC_Gripper> wsg;
	std::shared_ptr<VisionSystem> visionSystem;
	std::shared_ptr<Anyfeeder> anyfeeder;
	std::shared_ptr<robot_movement_interface::RobworkInterface> iiwa;

	std::shared_ptr<KukaIIWAPlanner> plannerTnt;
	std::shared_ptr<KukaIIWAPlanner> plannerTntRedundency;
	std::shared_ptr<Planner> planner;
	std::shared_ptr<Planner> plannerRedundency;

	Move move;
	Move2Q moveq;
	MoveLin movel;
	DynamicGrasp dg;

	ObjectType object_desired;
	ObjectType object_located;

};

#endif /* REVERSIBLE_DSL_SRC_SKILL_ANYFEEDERPICKSKILL_H_ */
