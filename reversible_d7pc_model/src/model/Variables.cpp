/*
 * Variables.cpp
 *
 *  Created on: Jun 20, 2017
 *      Author: josl
 */

#include <d7cp_proxys/gripper/D7PCGripper.h>
#include <d7cp_proxys/InspectionSystem.h>
#include <model/Variables.h>
#include <skill/AnyfeederPickSkill.h>
#include <d7cp_proxys/VisionSystem.h>

namespace model {

Variables::Variables() :
	frames("std::shared_ptr<rw::kinematics::Frame>"),
	q("rw::math::Q"),
	transform3D("rw::math::Transform3D<>"),
	graspType("gripper::D7PC_Gripper::GraspConfig"),
	visionType("VisionSystem::ObjectType"),
	inspectionType("InspectionSystem::ObjectType"),
	anyfeederType("AnyfeederPickSkill::ObjectType"),
	varString("Variable<std::string>"),
	varState("Variable<rw::kinematics::State>")

{
	graspType.save("innerStructure", gripper::D7PC_Gripper::inner_structure);
	graspType.save("outerShell", gripper::D7PC_Gripper::outer_shell);
	graspType.save("thermoElement", gripper::D7PC_Gripper::thermo_element);
	graspType.save("pin", gripper::D7PC_Gripper::pin);
	graspType.save("spring", gripper::D7PC_Gripper::spring);
	graspType.save("screwPart", gripper::D7PC_Gripper::skrew_part);

	visionType.save("outerShell", VisionSystem::ObjectType::OuterShell);
	visionType.save("innerStructure", VisionSystem::ObjectType::InnerStructure);
	visionType.save("screwPart", VisionSystem::ObjectType::ScrewPart);

	inspectionType.save("outerShell",InspectionSystem::ObjectType::OuterShell);
	inspectionType.save("innerStructure",InspectionSystem::ObjectType::InnerStructure);
	inspectionType.save("screwPart",InspectionSystem::ObjectType::ScrewPart);
	inspectionType.save("pin",InspectionSystem::ObjectType::Pin);

	anyfeederType.save("outerShell",AnyfeederPickSkill::ObjectType::OuterShell);
	anyfeederType.save("innerStructure",AnyfeederPickSkill::ObjectType::InnerStructure);
	anyfeederType.save("screwPart",AnyfeederPickSkill::ObjectType::ScrewPart);
}

Variables::~Variables()
{
}

} /* namespace model */
