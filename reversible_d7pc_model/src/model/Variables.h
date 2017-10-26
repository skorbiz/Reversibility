/*
 * Variables.h
 *
 *  Created on: Jun 20, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_VARIABLES_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_VARIABLES_H_

#include <d7cp_proxys/gripper/D7PCGripper.h>
#include <d7cp_proxys/InspectionSystem.h>
#include <model/MemoryModel.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <skill/AnyfeederPickSkill.h>
#include <d7cp_proxys/VisionSystem.h>
#include <model/Variable.h>

namespace model {

class Variables
{

public:
	Variables();
	virtual ~Variables();

	MemoryModel<std::shared_ptr<rw::kinematics::Frame> > frames;
	MemoryModel<rw::math::Q> q;
	MemoryModel<rw::math::Transform3D<> > transform3D;
	MemoryModel<gripper::D7PC_Gripper::GraspConfig>graspType;
	MemoryModel<VisionSystem::ObjectType> visionType;
	MemoryModel<InspectionSystem::ObjectType> inspectionType;
	MemoryModel<AnyfeederPickSkill::ObjectType> anyfeederType;

	MemoryModel<Variable<std::string> > varString;
	MemoryModel<Variable<rw::kinematics::State> > varState;


};

} /* namespace model */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_VARIABLES_H_ */





