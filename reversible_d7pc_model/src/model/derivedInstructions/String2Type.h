/*
 * String2Type.h
 *
 *  Created on: Jun 22, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_STRING2TYPE_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_STRING2TYPE_H_

#include <d7cp_proxys/gripper/D7PCGripper.h>
#include <d7cp_proxys/InspectionSystem.h>
#include <skill/AnyfeederPickSkill.h>
#include <d7cp_proxys/VisionSystem.h>
#include <string>

namespace model {

class String2Type
{
public:
	static gripper::D7PC_Gripper::GraspConfig toGraspConfig(std::shared_ptr<gripper::D7PC_Gripper> wsg, std::string name);
	static VisionSystem::ObjectType toVisionObjectType(std::string name);
	static InspectionSystem::ObjectType toInspectionObjectType(std::string name);
	static AnyfeederPickSkill::ObjectType toAnyfeederPickSkillObjectType(std::string name);
	static AnyfeederPickSkill::ObjectType toAnyfeederPickSkillObjectType(VisionSystem::ObjectType);
};

} /* namespace model */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_STRING2TYPE_H_ */
