/*
 * String2Type.cpp
 *
 *  Created on: Jun 22, 2017
 *      Author: josl
 */

#include <model/derivedInstructions/String2Type.h>
#include <cassert>
#include <cstdlib>
#include <iostream>


namespace model {

gripper::D7PC_Gripper::GraspConfig String2Type::toGraspConfig(std::shared_ptr<gripper::D7PC_Gripper> wsg, std::string name)
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

VisionSystem::ObjectType String2Type::toVisionObjectType(std::string name)
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

InspectionSystem::ObjectType String2Type::toInspectionObjectType(std::string name)
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

AnyfeederPickSkill::ObjectType String2Type::toAnyfeederPickSkillObjectType(std::string name)
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

AnyfeederPickSkill::ObjectType String2Type::toAnyfeederPickSkillObjectType(VisionSystem::ObjectType type)
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
} /* namespace model */
