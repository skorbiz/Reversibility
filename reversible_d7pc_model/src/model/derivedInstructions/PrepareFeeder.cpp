/*
 * PrepareFeeder.cpp
 *
 *  Created on: Jun 21, 2017
 *      Author: josl
 */

#include <model/derivedInstructions/PrepareFeeder.h>
#include <model/derivedInstructions/String2Type.h>
#include <model/MemoryModel.hpp>
#include <model/Variables.h>
#include <skill/AnyfeederPickSkill.h>
#include <cassert>
#include <iostream>

namespace model {

PrepareFeeder::PrepareFeeder(std::shared_ptr<AnyfeederPickSkill> interface, std::string variabel_object) :
		anyfeederPickSkill(interface),
		variabel_object(variabel_object)
{
}

PrepareFeeder::~PrepareFeeder()
{
}

ExecutionResult23 PrepareFeeder::execute()
{
	std::string obj = "";

	assert(variables != nullptr);
	if(variables->varString.doContain(variabel_object))
		obj = variables->varString.getRef(variabel_object).get();
	else
		std::cout << "Warning: PrepareFeeder::execute() did not find variable with name: " << variabel_object << std::endl;


	std::cout << __PRETTY_FUNCTION__ << " Got object: " << obj << std::endl;
	auto objType = String2Type::toAnyfeederPickSkillObjectType(obj);

//	anyfeederPickSkill->prepare(objType);

	ExecutionResult23 r;
	return r;

}

ExecutionResult23 PrepareFeeder::executeBackwards()
{
	std::cout << __PRETTY_FUNCTION__ << std::endl;
	return execute();
}


} /* namespace model */


/**
 * 	else if(command.command == "prepare feeder")
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
 */
